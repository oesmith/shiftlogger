#ifndef MBE_CAN_H
#define MBE_CAN_H

#include <stdint.h>

#include "freertos/FreeRTOS.h"

typedef struct {
  TickType_t ts;
  uint8_t raw_data[5];
  uint16_t rpm;
  float temp_c;
  uint8_t throttle;
  bool valid;
} mbe_can_data_t;

void mbe_can_init();
bool mbe_can_update(mbe_can_data_t* data);

#endif
