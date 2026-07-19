#ifndef TELEMETRY_H
#define TELEMETRY_H

#include <stdint.h>

void telemetry_init();
void telemetry_update(uint32_t event_ms, uint8_t* raw_can_data);

#endif // TELEMETRY_H
