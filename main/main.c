#include "nvs_flash.h"

#include "mbe_can.h"
#include "power.h"
#include "shiftlight.h"
#include "storage.h"
#include "telemetry.h"
#include "timesync.h"

#define TAG "shiftlogger"

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
  while (1) {
    bool has_time = timesync_update();
    bool has_new_data = mbe_can_update(poll_ts);
    bool has_power = power_has_power();
    bool has_sdcard = storage_has_card();

    if (has_new_data) {
      telemetry_update(mbe_can_raw_data());
    }

    if (has_new_data && has_time && has_sdcard) {
      storage_update(poll_ts, has_power, mbe_can_rpm(), mbe_can_temp_c(),
                     mbe_can_tps_site(), mbe_can_throttle());
    }

    uint8_t status = 0;
    if (mbe_can_is_data_valid()) {
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

    uint16_t rpm = mbe_can_rpm();
    float temp_c = mbe_can_temp_c();
    shiftlight_update(poll_ts, status, rpm, temp_c);

    xTaskDelayUntil(&poll_ts, pdMS_TO_TICKS(5));
  }
}
