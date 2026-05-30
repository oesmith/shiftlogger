#include "mbe_can.h"

#include "esp_log.h"
#include "esp_system.h"
#include "esp_twai.h"
#include "esp_twai_onchip.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define TAG "mbe_can"

#define MBE_ID_EASIMAP 0xcbe1101lu
#define MBE_ID_ECU 0xcbe0111lu

#define POLL_INTERVAL pdMS_TO_TICKS(50)
#define RECV_TIMEOUT pdMS_TO_TICKS(500)
#define DATA_VALIDITY_INTERVAL pdMS_TO_TICKS(2000)

// Resjacan
#define CAN_RX_GPIO 13
#define CAN_TX_GPIO 14
// Homebrew
// #define CAN_RX_GPIO 16
// #define CAN_TX_GPIO 17

// RPM - ( 0x7d, 0x7c )
// Coolant temp - ( 0x45, 0x44 )
// Voltage - ( 0x9f, 0x9e )
// TPS - ( 0x51, 0x50 )
// Throttle site - ( 0x64 )
#define QUERY_MSG_A {0x10, 0xb, 0x1, 0x0, 0x0, 0x0, 0x0, 0xf8}
#define QUERY_MSG_B {0x21, 0x7d, 0x7c, 0x45, 0x44, 0x64, 0x00}

static twai_node_handle_t twai_node = NULL;
static twai_onchip_node_config_t twai_node_config = {
    .io_cfg.rx = CAN_RX_GPIO,
    .io_cfg.tx = CAN_TX_GPIO,
    .io_cfg.bus_off_indicator = GPIO_NUM_NC,
    .io_cfg.quanta_clk_out = GPIO_NUM_NC,
    .bit_timing.bitrate = 500000,
    .tx_queue_depth = 5,
};

static QueueHandle_t rx_queue;

static uint16_t rpm;
static float temp_c;
static uint8_t raw_data[6] = {0};

static TickType_t zero_ts;
static TickType_t recv_ts;
static TickType_t recv_timeout_ts;
static TickType_t send_ts;
static TickType_t next_send_ts;

static bool twai_rx_cb(twai_node_handle_t handle,
                       const twai_rx_done_event_data_t* edata, void* user_ctx) {
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  uint8_t buf[8] = {0};
  twai_frame_t rx_frame = {
      .buffer = buf,
      .buffer_len = 8,
  };
  if (ESP_OK == twai_node_receive_from_isr(handle, &rx_frame)) {
    xQueueSendToBackFromISR(rx_queue, &buf, &xHigherPriorityTaskWoken);
  }
  if (xHigherPriorityTaskWoken == pdTRUE) {
    portYIELD_FROM_ISR();
  }
  return false;
}

static bool twai_state_cb(twai_node_handle_t handle,
                          const twai_state_change_event_data_t* edata,
                          void* user_ctx) {
  const char* twai_state_name[] = {"error_active", "error_warning",
                                   "error_passive", "bus_off"};
  ESP_EARLY_LOGI(TAG, "state changed: %s -> %s",
                 twai_state_name[edata->old_sta],
                 twai_state_name[edata->new_sta]);
  return false;
}

static bool twai_error_cb(twai_node_handle_t handle,
                          const twai_error_event_data_t* edata,
                          void* user_ctx) {
  ESP_EARLY_LOGW(TAG, "bus error: 0x%x", edata->err_flags.val);
  return false;
}

void mbe_can_init() {
  rx_queue = xQueueCreate(5, 8);
  if (rx_queue == NULL) {
    ESP_LOGE(TAG, "Failed to create recv queue");
    return;
  }

  ESP_ERROR_CHECK(twai_new_node_onchip(&twai_node_config, &twai_node));

  twai_mask_filter_config_t mfilter_cfg = {
      .id = MBE_ID_ECU, .mask = TWAI_EXT_ID_MASK, .is_ext = true};
  ESP_ERROR_CHECK(twai_node_config_mask_filter(
      twai_node, 0, &mfilter_cfg));  // Configure on filter 0

  twai_event_callbacks_t user_cbs = {
      .on_rx_done = twai_rx_cb,
      .on_state_change = twai_state_cb,
      .on_error = twai_error_cb,
  };
  ESP_ERROR_CHECK(
      twai_node_register_event_callbacks(twai_node, &user_cbs, NULL));

  ESP_ERROR_CHECK(twai_node_enable(twai_node));

  zero_ts = recv_ts = recv_timeout_ts = send_ts = next_send_ts =
      xTaskGetTickCount();
}

esp_err_t send_query() {
  esp_err_t ret;
  uint8_t buf_a[8] = QUERY_MSG_A;
  uint8_t buf_b[8] = QUERY_MSG_B;
  twai_frame_t frame_a = {
      .header.id = MBE_ID_EASIMAP,
      .header.ide = true,
      .buffer = buf_a,
      .buffer_len = sizeof(buf_a),
  };
  twai_frame_t frame_b = {
      .header.id = MBE_ID_EASIMAP,
      .header.ide = true,
      .buffer = buf_b,
      .buffer_len = sizeof(buf_b),
  };
  // Send both frames, then wait for transmission to complete.
  if ((ret = twai_node_transmit(twai_node, &frame_a, 0)) != ESP_OK) {
    ESP_LOGE(TAG, "Send failed (frame A): %s", esp_err_to_name(ret));
  } else if ((ret = twai_node_transmit(twai_node, &frame_b, 0)) != ESP_OK) {
    ESP_LOGE(TAG, "Send failed (frame B): %s", esp_err_to_name(ret));
  } else if ((ret = twai_node_transmit_wait_all_done(twai_node, -1)) !=
             ESP_OK) {
    ESP_LOGE(TAG, "Send failed (wait): %s", esp_err_to_name(ret));
  }
  return ret;
}

esp_err_t recv_response() {
  uint8_t buf[8] = {0};
  BaseType_t ret = xQueueReceive(rx_queue, &buf, 0 /* don't block */);
  if (ret == pdPASS) {
    rpm = 0x100 * buf[2] + buf[3];
    temp_c = (0x100 * buf[4] + buf[5]) * 160.0f / 65535.0f - 30.0f;
    memcpy(raw_data, &buf[2], 5);
    return ESP_OK;
  } else {  // (ret == errQUEUE_EMPTY)
    return ESP_ERR_TIMEOUT;
  }
}

bool mbe_can_update(TickType_t ts) {
  bool has_new_data = false;
  if (recv_response() == ESP_OK) {
    recv_ts = ts;
    has_new_data = true;
  }

  bool should_send =
      ts >= recv_timeout_ts || (recv_ts >= send_ts && ts >= next_send_ts);
  if (should_send && send_query() == ESP_OK) {
    send_ts = ts;
    next_send_ts = ts + POLL_INTERVAL;
    recv_timeout_ts = ts + RECV_TIMEOUT;
  }

  return has_new_data;
}

bool mbe_can_is_data_valid() {
  return recv_ts > zero_ts &&
         (recv_ts >= send_ts || (send_ts - recv_ts) < DATA_VALIDITY_INTERVAL);
}

uint16_t mbe_can_rpm() { return rpm; }

float mbe_can_temp_c() { return temp_c; }

uint8_t* mbe_can_raw_data() { return raw_data; }
