#include "blackbox_ring.h"
#include <string.h>

static uint8_t bb_buf[BB_RING_SZ];
static uint32_t bb_head = 0;
static uint32_t bb_total = 0;
static bool bb_wrapped = false;

void bb_ring_init(void) {
    memset(bb_buf, 0, BB_RING_SZ);
    bb_head = 0;
    bb_total = 0;
    bb_wrapped = false;
}

void bb_ring_push(const void *data, uint32_t len) {
    const uint8_t *p = (const uint8_t *)data;
    for (uint32_t i = 0; i < len; i++) {
        bb_buf[bb_head] = p[i];
        bb_head = (bb_head + 1) % BB_RING_SZ;
        bb_total++;
        if (bb_total > BB_RING_SZ) bb_wrapped = true;
    }
}

void bb_ring_event(uint8_t ev) {
    uint8_t m[3] = {0xEE, ev, 0xFF};
    bb_ring_push(m, 3);
}

void bb_ring_dump(void) {
    /* TODO: write to flash */
}
