#ifndef SHIFTLIGHT_H
#define SHIFTLIGHT_H

#include "freertos/FreeRTOS.h"

#define SHIFTLIGHT_NO_DATA_RPM 0xffff

void shiftlight_init();
void shiftlight_update(TickType_t ts, uint16_t rpm, float temp_c);

#endif  // SHIFTLIGHT_H