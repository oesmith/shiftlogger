#ifndef MBE_CAN_H
#define MBE_CAN_H

#include <stdint.h>

#include "freertos/FreeRTOS.h"

void mbe_can_init();
bool mbe_can_update(TickType_t ts);
bool mbe_can_is_data_valid();
uint16_t mbe_can_rpm();
float mbe_can_temp_c();
float mbe_can_tps_site();
uint8_t mbe_can_throttle();
uint8_t* mbe_can_raw_data();

#endif
