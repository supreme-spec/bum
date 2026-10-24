#include "radun.h"

static uint8_t crsf_buf[64];
static uint8_t crsf_pos = 0;
static crsf_t crsf_data = {0};

static uint8_t crsf_crc8(const uint8_t* data, uint8_t len) {
    uint8_t crc = 0;
    for (uint8_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int j = 0; j < 8; j++) {
            if (crc & 0x80) crc = (crc << 1) ^ 0xD5;
            else crc <<= 1;
        }
    }
    return crc;
}

void crsf_init(void) {
    crsf_data.lq = 0xFF;
    RCC->APB2 |= (1 << 5);
    gpio_af(GPIOC, GPIO_PIN_7, 8);
    gpio_af(GPIOC, GPIO_PIN_6, 8);
    uart_init(USART6, 420000);
}

void crsf_poll(crsf_t* out) {
    while (USART6->STS & (1 << 4)) {
        uint8_t b = USART6->DT;
        if (b == 0xC8) crsf_pos = 0;
        if (crsf_pos < 64) crsf_buf[crsf_pos++] = b;
        if (crsf_pos >= 26 && crsf_buf[2] == 0x16) {
            uint8_t expected_len = crsf_buf[1];
            if (crsf_pos >= expected_len + 4) {
                uint8_t crc = crsf_crc8(&crsf_buf[2], expected_len - 1);
                if (crc == crsf_buf[2 + expected_len - 1]) {
                    uint32_t raw = 0;
                    for (int i = 0; i < 6; i++) raw |= ((uint32_t)crsf_buf[3 + i]) << (i*8);
                    crsf_data.r = (int16_t)((raw & 0x7FF) - 992);
                    crsf_data.p = (int16_t)(((raw >> 11) & 0x7FF) - 992);
                    crsf_data.thr = (int16_t)(((raw >> 22) & 0x7FF));
                    crsf_data.y = (int16_t)(((raw >> 33) & 0x7FF) - 992);
                    crsf_data.aux = (raw >> 44) & 0xFFFF;
                    crsf_data.lq = 0xFF;
                    crsf_data.valid = true;
                }
                crsf_pos = 0;
            }
        }
    }
    *out = crsf_data;
}