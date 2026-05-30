#ifndef SHIFTLIGHT_H
#define SHIFTLIGHT_H

#include "freertos/FreeRTOS.h"

#define SHIFTLIGHT_STATUS_HAS_CAN_DATA 0x1
#define SHIFTLIGHT_STATUS_HAS_SD_CARD  0x2
#define SHIFTLIGHT_STATUS_HAS_TIME     0x4
#define SHIFTLIGHT_STATUS_IS_RECORDING 0x8
#define SHIFTLIGHT_STATUS_HAS_POWER    0x10

void shiftlight_init();
void shiftlight_update(TickType_t ts, uint8_t status, uint16_t rpm, float temp_c);

#endif  // SHIFTLIGHT_H
