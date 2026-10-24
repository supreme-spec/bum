#include "radun.h"

static bool usb_cdc_initialized = false;
static bool usb_cdc_connected = false;

void usb_cdc_init(void) {
    usb_cdc_initialized = true;   /* полноценный USB CDC — на этапе P1+ */
}

bool usb_cdc_is_connected(void) {
    return usb_cdc_connected;
}

void usb_cdc_send(const uint8_t *data, uint16_t len) {
    (void)data; (void)len;
}

uint16_t usb_cdc_receive(uint8_t *data, uint16_t max_len) {
    (void)data; (void)max_len;
    return 0;
}

void usb_cdc_poll(void) {
}
