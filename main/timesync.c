#include <time.h>
#include <sys/time.h>
#include "esp_log.h"
#include "driver/uart.h"

#define TAG "timesync"

#define GPS_EPOCH (315964800)

// Wait at least 15s after startup for GPS sync.
#define MIN_SYNC_TICKS (pdMS_TO_TICKS(15000))

static const uint8_t CFG_MSG_TEMPLATE[] = {
  0xB5, 0x62, // Magic
  0x06, // Class -- 0x06 CFG
  0x01, // ID    -- 0x01 MSG
  0x03, 0x00, // Payload length
        0x00, // Class
        0x00, // ID
        0x01, // Rate
  0x00, 0x00 // Checksum
};

static bool has_sync = false;
static QueueHandle_t serial_queue;

void handle_data(size_t size);
void parse_nav_time_gps(uint8_t *data, size_t len);
void ubx_config_msg(uint8_t class, uint8_t id, uint8_t rate);
uint32_t millis_since_midnight(struct tm* tm, int usec);

void timesync_init(void) {
  ESP_ERROR_CHECK(
      uart_driver_install(UART_NUM_1, 1024, 1024, 10, &serial_queue, 0));
  uart_config_t cfg = {
      .baud_rate = 9600,
      .data_bits = UART_DATA_8_BITS,
      .parity = UART_PARITY_DISABLE,
      .stop_bits = UART_STOP_BITS_1,
      .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
  };
  ESP_ERROR_CHECK(uart_param_config(UART_NUM_1, &cfg));
  ESP_ERROR_CHECK(uart_set_pin(UART_NUM_1, 48, 47, UART_PIN_NO_CHANGE,
                               UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE,
                               UART_PIN_NO_CHANGE));

  ubx_config_msg(0xf0, 0x00, 0); // Disable GGA
  ubx_config_msg(0xf0, 0x01, 0); // Disable GLL
  ubx_config_msg(0xf0, 0x02, 0); // Disable GSA
  ubx_config_msg(0xf0, 0x03, 0); // Disable GSV
  ubx_config_msg(0xf0, 0x04, 0); // Disable RMC
  ubx_config_msg(0xf0, 0x05, 0); // Disable VTG
  ubx_config_msg(0xf0, 0x05, 0); // Disable TXT
  ubx_config_msg(0x01, 0x20, 1); // Enable TIMEGPS
}

bool timesync_update(void) {
  uart_event_t event;
  while (!has_sync && xQueueReceive(serial_queue, &event, 0)) {
    switch (event.type) {
    case UART_DATA:
      handle_data(event.size);
      break;
    case UART_FIFO_OVF:
      ESP_LOGW(TAG, "FIFO overflow");
      uart_flush(UART_NUM_1);
      xQueueReset(serial_queue);
      break;
    case UART_BUFFER_FULL:
      ESP_LOGW(TAG, "Buffer full");
      uart_flush(UART_NUM_1);
      xQueueReset(serial_queue);
      break;
    case UART_BREAK:
      ESP_LOGW(TAG, "UART break");
      break;
    case UART_PARITY_ERR:
      ESP_LOGW(TAG, "UART parity error");
      break;
    case UART_FRAME_ERR:
      ESP_LOGW(TAG, "UART frame error");
      break;
    case UART_PATTERN_DET:
      // No-op;
      break;
    default:
      ESP_LOGW(TAG, "Unknown UART event: %d", event.type);
      break;
    }
  }
  return has_sync;
}

void handle_data(size_t size) {
  uint8_t buf[64];
  while (!has_sync && size > 0) {
    size_t n = size > sizeof(buf) ? sizeof(buf) : size;
    size -= n;
    uart_read_bytes(UART_NUM_1, buf, n, 0);
    parse_nav_time_gps(buf, n);
  }
}

void parse_nav_time_gps(uint8_t *data, size_t len) {
  if (len != 24 ||
      data[0] != 0xb5 ||
      data[1] != 0x62 ||
      data[2] != 0x01 ||
      data[3] != 0x20 ||
      data[4] != 0x10 ||
      data[5] != 0x00) {
    return;
  }

  uint32_t gps_tow =
    ((uint32_t)data[6])
    + (((uint32_t)data[7]) << 8)
    + (((uint32_t)data[8]) << 16)
    + (((uint32_t)data[9]) << 24);
  uint16_t gps_week =
    ((uint16_t)data[14])
    + (((uint16_t)data[15]) << 8);
  uint8_t leap_seconds = data[16];
  uint8_t flags = data[17];

  ESP_LOGI(TAG, "Week %d, Time of week %dms, %d leap seconds, flags %d", gps_week, gps_tow, leap_seconds, flags);

  if ((flags & 0x4) == 0) {
    // If we don't have UTC leap seconds from GPS yet, then assume we're in
    // 2026 where the offset is 18 seconds.
    leap_seconds = 18;
  }

  time_t ts =
    GPS_EPOCH
    + 7 * 86400 * (time_t)gps_week
    + (time_t)gps_tow / 1000
    - (time_t)leap_seconds;

  TickType_t ticks = xTaskGetTickCount();

  if (ticks >= MIN_SYNC_TICKS && (flags & 0x3) == 0x3) {
    struct timeval tv = {
      .tv_sec = ts,
      .tv_usec = 0
    };
    settimeofday(&tv, NULL);

    has_sync = true;
    uart_driver_delete(UART_NUM_1);
  }

  struct tm tm;
  gmtime_r(&ts, &tm);
  char sz[64] = {0};
  strftime(sz, sizeof(sz), "%Y-%m-%d %H:%M:%S", &tm);
  ESP_LOGI(TAG, "Date %s", sz);
}

void ubx_config_msg(uint8_t class, uint8_t id, uint8_t rate) {
  uint8_t req[sizeof(CFG_MSG_TEMPLATE)] = {0};
  memcpy(req, CFG_MSG_TEMPLATE, sizeof(CFG_MSG_TEMPLATE));
  req[6] = class;
  req[7] = id;
  req[8] = rate;
  uint8_t a = 0, b = 0;
  for (int i = 2; i < sizeof(req)-2; i++) {
    a += req[i];
    b += a;
  }
  req[sizeof(req) - 2] = a;
  req[sizeof(req) - 1] = b;
  uart_write_bytes(UART_NUM_1, req, sizeof(req));
}

uint32_t millis_since_midnight(struct tm *tm, int usec) {
  return tm->tm_hour * 60 * 60 * 1000 // Hours
         + tm->tm_min * 60 * 1000     // Minutes
         + tm->tm_sec * 1000          // Seconds
         + (usec / 1000);             // Milliseconds
}

uint32_t timesync_millis(TickType_t ts) {
  TickType_t now_ts = xTaskGetTickCount();

  struct timeval tv;
  gettimeofday(&tv, NULL);

  struct tm tm;
  gmtime_r(&tv.tv_sec, &tm);

  return millis_since_midnight(&tm, tv.tv_usec) - (now_ts - ts);
}
