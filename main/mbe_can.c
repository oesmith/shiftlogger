#include "mbe_can.h"

#include "driver/gptimer.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_twai.h"
#include "esp_twai_onchip.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <math.h>

#define TAG "mbe_can"

#define MBE_ID_EASIMAP 0xcbe1101lu
#define MBE_ID_ECU 0xcbe0111lu

// APB timer is 80mhz, so each timer tick is 1us
#define TIMER_DIVIDER (80)

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

static gptimer_handle_t timer;

static TickType_t send_ts;
static bool response_pending = false;

typedef struct {
  uint8_t buf[8];
  TickType_t req_ts;
  TickType_t res_ts;
} recv_msg_t;

static bool timer_cb(gptimer_handle_t t, const gptimer_alarm_event_data_t *d,
                     void *ctx) {
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;

  // Send the request frames on every timer interrupt.
  static const uint8_t buf_a[8] = QUERY_MSG_A;
  static const uint8_t buf_b[8] = QUERY_MSG_B;
  static const twai_frame_t frame_a = {
      .header.id = MBE_ID_EASIMAP,
      .header.ide = true,
      .buffer = (uint8_t *)buf_a,
      .buffer_len = sizeof(buf_a),
  };
  static const twai_frame_t frame_b = {
      .header.id = MBE_ID_EASIMAP,
      .header.ide = true,
      .buffer = (uint8_t *)buf_b,
      .buffer_len = sizeof(buf_b),
  };

  if (response_pending) {
    recv_msg_t recv_msg = {
        .buf = {0},
        .req_ts = send_ts,
        .res_ts = 0,
    };
    xQueueSendToBackFromISR(rx_queue, &recv_msg, &xHigherPriorityTaskWoken);
  }

  twai_node_transmit(twai_node, &frame_a, 0);
  twai_node_transmit(twai_node, &frame_b, 0);

  send_ts = xTaskGetTickCountFromISR();
  response_pending = true;

  return xHigherPriorityTaskWoken;
}

static bool twai_rx_cb(twai_node_handle_t handle,
                       const twai_rx_done_event_data_t *edata, void *user_ctx) {
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  TickType_t ts = xTaskGetTickCountFromISR();
  recv_msg_t recv_msg = {
      .buf = {0},
      .req_ts = send_ts,
      .res_ts = ts,
  };
  twai_frame_t rx_frame = {
      .buffer = recv_msg.buf,
      .buffer_len = 8,
  };
  if (ESP_OK == twai_node_receive_from_isr(handle, &rx_frame)) {
    response_pending = false;
    xQueueSendToBackFromISR(rx_queue, &recv_msg, &xHigherPriorityTaskWoken);
  }
  if (xHigherPriorityTaskWoken == pdTRUE) {
    portYIELD_FROM_ISR();
  }
  return false;
}

static bool twai_state_cb(twai_node_handle_t handle,
                          const twai_state_change_event_data_t *edata,
                          void *user_ctx) {
  const char *twai_state_name[] = {"error_active", "error_warning",
                                   "error_passive", "bus_off"};
  ESP_EARLY_LOGI(TAG, "state changed: %s -> %s",
                 twai_state_name[edata->old_sta],
                 twai_state_name[edata->new_sta]);
  return false;
}

static bool twai_error_cb(twai_node_handle_t handle,
                          const twai_error_event_data_t *edata,
                          void *user_ctx) {
  // ESP_EARLY_LOGW(TAG, "bus error: 0x%x", edata->err_flags.val);
  return false;
}

void mbe_can_init() {
  rx_queue = xQueueCreate(5, sizeof(recv_msg_t));
  if (rx_queue == NULL) {
    ESP_LOGE(TAG, "Failed to create recv queue");
    return;
  }

  ESP_ERROR_CHECK(twai_new_node_onchip(&twai_node_config, &twai_node));

  twai_mask_filter_config_t mfilter_cfg = {
      .id = MBE_ID_ECU, .mask = TWAI_EXT_ID_MASK, .is_ext = true};
  ESP_ERROR_CHECK(twai_node_config_mask_filter(
      twai_node, 0, &mfilter_cfg)); // Configure on filter 0

  twai_event_callbacks_t user_cbs = {
      .on_rx_done = twai_rx_cb,
      .on_state_change = twai_state_cb,
      .on_error = twai_error_cb,
  };
  ESP_ERROR_CHECK(
      twai_node_register_event_callbacks(twai_node, &user_cbs, NULL));

  ESP_ERROR_CHECK(twai_node_enable(twai_node));

  gptimer_config_t timer_cfg = {
      .clk_src = GPTIMER_CLK_SRC_DEFAULT,
      .direction = GPTIMER_COUNT_UP,
      .resolution_hz = 1000000, // 1MHz clock -- 1us resolution.
  };
  ESP_ERROR_CHECK(gptimer_new_timer(&timer_cfg, &timer));

  gptimer_alarm_config_t alarm_cfg = {
      .reload_count = 0,
      .alarm_count = 50000, // 50,000us interval -- 20Hz alert.
      .flags.auto_reload_on_alarm = true,
  };
  ESP_ERROR_CHECK(gptimer_set_alarm_action(timer, &alarm_cfg));

  gptimer_event_callbacks_t cb_cfg = {
      .on_alarm = timer_cb,
  };
  ESP_ERROR_CHECK(gptimer_register_event_callbacks(timer, &cb_cfg, NULL));

  ESP_ERROR_CHECK(gptimer_enable(timer));
  ESP_ERROR_CHECK(gptimer_start(timer));

  send_ts = xTaskGetTickCount();
}

bool mbe_can_update(mbe_can_data_t *out) {
  recv_msg_t msg;
  if (xQueueReceive(rx_queue, &msg, 0 /* don't block */) != pdPASS) {
    return false;
  }
  out->ts = msg.req_ts;
  out->rpm = 0x100 * msg.buf[2] + msg.buf[3];
  out->temp_c = (0x100 * msg.buf[4] + msg.buf[5]) * 160.0f / 65535.0f - 30.0f;
  out->throttle = (uint8_t)roundf(
      100.0f * fmaxf(fminf(msg.buf[6] * 16.0f / 255.0f, 15.0f), 0.0f) / 15.0f);
  out->valid = (msg.res_ts != 0);
  memcpy(out->raw_data, &msg.buf[2], 5);
  return true;
}
