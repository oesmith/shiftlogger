#include "nvs_flash.h"

#include "mbe_can.h"
#include "power.h"
#include "shiftlight.h"
#include "storage.h"
#include "telemetry.h"
#include "timesync.h"

#define TAG "shiftlogger"

#define DATA_VALIDITY_INTERVAL pdMS_TO_TICKS(250)

void nvs_init(void) {
  /* Initialize NVS — it is used to store WiFi/BLE calibration data */
  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
      ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
  }
  ESP_ERROR_CHECK(ret);
}

void app_main(void) {
  nvs_init();

  power_init();
  mbe_can_init();
  shiftlight_init();
  telemetry_init();
  storage_init();
  timesync_init();

  TickType_t poll_ts = xTaskGetTickCount();
  mbe_can_data_t can_data;
  while (1) {
    bool has_time = timesync_update();
    bool has_new_data = mbe_can_update(&can_data);
    bool has_power = power_has_power();
    bool has_sdcard = storage_has_card();

    uint32_t event_ms =
        (has_new_data && has_time) ? timesync_millis(can_data.ts) : 0;

    if (has_new_data) {
      telemetry_update(event_ms, can_data.raw_data);
    }

    if (has_new_data && has_time && has_sdcard) {
      storage_update(can_data.ts, has_power, can_data.rpm, can_data.temp_c,
                     can_data.throttle, event_ms);
    }

    uint8_t status = 0;
    if (can_data.valid && (poll_ts - can_data.ts) < DATA_VALIDITY_INTERVAL) {
      status |= SHIFTLIGHT_STATUS_HAS_CAN_DATA;
    }

    if (has_time) {
      status |= SHIFTLIGHT_STATUS_HAS_TIME;
    }

    if (has_power) {
      status |= SHIFTLIGHT_STATUS_HAS_POWER;
    }

    if (has_sdcard) {
      status |= SHIFTLIGHT_STATUS_HAS_SD_CARD;
    }

    bool is_recording = storage_is_recording();
    power_force_on(is_recording);
    if (is_recording) {
      status |= SHIFTLIGHT_STATUS_IS_RECORDING;
    }

    shiftlight_update(poll_ts, status, can_data.rpm, can_data.temp_c);

    xTaskDelayUntil(&poll_ts, pdMS_TO_TICKS(20));
  }
}
