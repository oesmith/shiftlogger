#ifndef SHIFTLIGHT_H
#define SHIFTLIGHT_H

#include "freertos/FreeRTOS.h"

#define SHIFTLIGHT_STATUS_HAS_CAN_DATA 0x1
#define SHIFTLIGHT_STATUS_HAS_POWER    0x2
#define SHIFTLIGHT_STATUS_HAS_CLIENT   0x4

void shiftlight_init();
void shiftlight_update(TickType_t ts, uint8_t status, uint16_t rpm, float temp_c);

#endif  // SHIFTLIGHT_H
