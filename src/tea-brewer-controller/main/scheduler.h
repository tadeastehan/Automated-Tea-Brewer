#ifndef SCHEDULER_H
#define SCHEDULER_H

#include <stdint.h>
#include <stdbool.h>

void scheduler_init(void);
void scheduler_set(uint8_t hour, uint8_t minute, uint8_t target_temp, uint8_t brewing_temp);
void scheduler_cancel(void);
bool scheduler_is_active(void);
void scheduler_get_status(uint32_t *target_time, uint32_t *remaining_sec);

#endif
