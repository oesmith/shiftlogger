#include <time.h>
#include <sys/time.h>
#include "esp_log.h"
#include "driver/uart.h"

#define TAG "timesync"

static bool has_sync = false;
static QueueHandle_t serial_queue;

void handle_line(void);
void decode_gprmc(char *buf);

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
  ESP_ERROR_CHECK(
      uart_enable_pattern_det_baud_intr(UART_NUM_1, '\n', 1, 9, 0, 0));
}

bool timesync_update(void) {
  uart_event_t event;
  if (xQueueReceive(serial_queue, &event, 0)) {
    switch (event.type) {
    case UART_DATA:
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
      handle_line();
      break;
    default:
      ESP_LOGW(TAG, "Unknown UART event: %d", event.type);
      break;
    }
  }
  return has_sync;
}

void handle_line(void) {
  char buf[1024];
  int pos = uart_pattern_pop_pos(UART_NUM_1);
  if (pos == -1) {
    ESP_LOGW(TAG, "Pattern too small");
    uart_flush_input(UART_NUM_1);
    return;
  }
  int read_len = uart_read_bytes(UART_NUM_1, buf, pos + 1, 0);
  buf[read_len] = 0;
  if (strnstr(buf, "$GPRMC", read_len) == buf) {
    decode_gprmc(buf);
  }
}

void decode_gprmc(char *buf) {
  if (has_sync) {
    return;
  }

  if (1) {
    ESP_LOGI(TAG, "RECV: %s", buf);
  }

  char *time = NULL;
  char *fix = NULL;
  char *date = NULL;

  char *tok;
  for (int i = 1; buf != NULL; i++) {
    tok = strsep(&buf, ",");
    if (i == 2) {
      time = tok;
    } else if (i == 3) {
      fix = tok;
    } else if (i == 10) {
      date = tok;
    }
  }

  if (time == NULL || fix == NULL || date == NULL) {
    return;
  }

  size_t time_len = strlen(time);
  size_t fix_len = strlen(fix);
  size_t date_len = strlen(date);
  if (time_len == 0 || fix_len == 0 || date_len == 0) {
    return;
  }

  if (time_len != 9) {
    ESP_LOGW(TAG, "Invalid time string: %s", time);
    return;
  }
  if (fix_len != 1) {
    ESP_LOGW(TAG, "Invalid fix string: %s", fix);
    return;
  }
  if (date_len != 6) {
    ESP_LOGW(TAG, "Invalid date string: %s", date);
    return;
  }

  if (fix[0] != 'A') {
    return;
  }

  struct tm tm = {0};
  int msec = 0;

  // Parse time components.
  tm.tm_hour = (int)(time[0] - '0') * 10 + (time[1] - '0');
  tm.tm_min = (int)(time[2] - '0') * 10 + (time[3] - '0');
  tm.tm_sec = (int)(time[4] - '0') * 10 + (time[5] - '0');
  msec = (int)(time[7] - '0') * 100 + (int)(time[8] - '0') * 10;

  // Parse date components.
  tm.tm_year = 100 + (int)(date[4] - '0') * 10 + (date[5] - '0');
  tm.tm_mon = (int)(date[2] - '0') * 10 + (date[3] - '0') - 1;
  tm.tm_mday = (int)(date[0] - '0') * 10 + (date[1] - '0');

  char ts[64];
  strftime(ts, sizeof(ts), "%Y-%m-%d %H:%M:%S", &tm);

  struct timeval tv = {
    .tv_sec = timegm(&tm),
    .tv_usec = msec * 1000,
  };
  settimeofday(&tv, NULL);

  ESP_LOGI(TAG, "Timestamp: (%s) %s.%03d", fix, ts, msec);
  has_sync = true;
}
