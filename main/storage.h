#ifndef STORAGE_H
#define STORAGE_H

#include "freertos/FreeRTOS.h"
#include <stdint.h>

void storage_init(void);
void storage_update(TickType_t ts, bool has_power, uint16_t rpm, float temp_c,
                    uint8_t throttle, uint32_t event_ms);
bool storage_has_card(void);
bool storage_is_recording(void);

#endif // STORAGE_H
