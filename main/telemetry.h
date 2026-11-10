#ifndef TELEMETRY_H
#define TELEMETRY_H

#include <stdint.h>

void telemetry_init();
void telemetry_update(uint8_t* raw_can_data);
bool telemetry_active();

#endif // TELEMETRY_H
