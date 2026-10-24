#pragma once

#include <stdbool.h>
#include <stdint.h>

void watchdog_init(uint32_t timeout_ms);
void watchdog_feed(void);
bool watchdog_was_reset(void);
