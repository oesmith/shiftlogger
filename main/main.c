#include "mbe_can.h"
#include "shiftlight.h"
#include "telemetry.h"

#define TAG "shiftlogger"

void app_main(void) {
  mbe_can_init();
  shiftlight_init();

  telemetry_init();

  TickType_t poll_ts = xTaskGetTickCount();
  while (1) {
    bool has_data = mbe_can_update(poll_ts);

    if (has_data && mbe_can_is_data_valid()) {
      telemetry_update(mbe_can_raw_data());
    }

    if (mbe_can_is_data_valid()) {
      shiftlight_update(poll_ts, mbe_can_rpm(), mbe_can_temp_c());
    } else {
      shiftlight_update(poll_ts, SHIFTLIGHT_NO_DATA_RPM, 0.0f);
    }

    xTaskDelayUntil(&poll_ts, pdMS_TO_TICKS(5));
  }
}
