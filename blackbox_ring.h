#pragma once
#include <stdint.h>
#include <stdbool.h>

#define BB_RING_SZ (96 * 1024)

void bb_ring_init(void);
void bb_ring_push(const void *data, uint32_t len);
void bb_ring_event(uint8_t ev);
void bb_ring_dump(void);
