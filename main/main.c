#include <time.h>
#include <sys/time.h>
#include "nvs_flash.h"

#include "mbe_can.h"
#include "power.h"
#include "shiftlight.h"
#include "storage.h"
#include "telemetry.h"

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

static bool cached_has_time = false;

bool get_has_time() {
  if (cached_has_time) {
    return true;
  }
  time_t t = time(NULL);
  cached_has_time = t > 1735689600;
  return cached_has_time;
}

uint32_t millis_since_midnight(struct tm *tm, int usec) {
  return tm->tm_hour * 60 * 60 * 1000 // Hours
         + tm->tm_min * 60 * 1000     // Minutes
         + tm->tm_sec * 1000          // Seconds
         + (usec / 1000);             // Milliseconds
}

uint32_t tick_millis(TickType_t ts) {
  TickType_t now_ts = xTaskGetTickCount();

  struct timeval tv;
  gettimeofday(&tv, NULL);

  struct tm tm;
  gmtime_r(&tv.tv_sec, &tm);

  return millis_since_midnight(&tm, tv.tv_usec) - (now_ts - ts);
}

void app_main(void) {
  nvs_init();

  power_init();
  mbe_can_init();
  shiftlight_init();
  telemetry_init();
  storage_init();

  TickType_t poll_ts = xTaskGetTickCount();
  mbe_can_data_t can_data;
  while (1) {
    bool has_time = get_has_time();
    bool has_new_data = mbe_can_update(&can_data);
    bool has_power = power_has_power();
    bool has_sdcard = storage_has_card();

    uint32_t event_ms =
        (has_new_data && has_time) ? tick_millis(can_data.ts) : 0;

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
