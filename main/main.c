#include "mbe_can.h"
#include "shiftlight.h"
#include "telemetry.h"
#include "vbox.h"

#define TAG "shiftlogger"

void app_main(void) {
  mbe_can_init();
  shiftlight_init();
  // TODO: get this all working.
  // vbox_init();
  // telemetry_init();

  TickType_t poll_ts = xTaskGetTickCount();
  while (1) {
    mbe_can_update(poll_ts);

    if (mbe_can_is_data_valid()) {
      shiftlight_update(poll_ts, mbe_can_rpm(), mbe_can_temp_c());
    } else {
      shiftlight_update(poll_ts, SHIFTLIGHT_NO_DATA_RPM, 0.0f);
    }

    xTaskDelayUntil(&poll_ts, pdMS_TO_TICKS(20));
  }
}