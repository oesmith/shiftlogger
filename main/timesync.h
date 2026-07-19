#ifndef TIMESYNC_H
#define TIMESYNC_H

#include "FreeRTOS/FreeRTOS.h"

void timesync_init(void);
bool timesync_update(void);
uint32_t timesync_millis(TickType_t ts);

#endif // TIMESYNC_H
