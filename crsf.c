/* crsf.c — полный парсер всех 16 каналов CRSF */
#include "radun.h"

static uint8_t crsf_buf[64];
static uint8_t crsf_pos = 0;
static crsf_t crsf_data = {0};
static uint32_t last_frame_ms = 0;
static uint16_t channels[16] = {0};

static uint8_t crsf_crc8(const uint8_t *data, uint8_t len) {
    uint8_t crc = 0;
    for (uint8_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int j = 0; j < 8; j++) {
            if (crc & 0x80) crc = (crc << 1) ^ 0xD5;
            else          crc <<= 1;
        }
    }
    return crc;
}

static uint16_t extract_11bit(const uint8_t *payload, int bit_offset) {
    int byte_offset = bit_offset / 8;
    int bit_in_byte = bit_offset % 8;
    uint32_t raw = 0;
    raw |= (uint32_t)payload[byte_offset + 0] << 0;
    raw |= (uint32_t)payload[byte_offset + 1] << 8;
    raw |= (uint32_t)payload[byte_offset + 2] << 16;
    return (raw >> bit_in_byte) & 0x7FF;
}

void crsf_init(void) {
    crsf_data.lq = 0xFF;
    RCC->APB2 |= (1 << 5);
    gpio_af(GPIOC, GPIO_PIN_7, 8);
    gpio_af(GPIOC, GPIO_PIN_6, 8);
    uart_init(USART6, 420000);
}

void crsf_poll(crsf_t *out) {
    while (USART6->STS & (1 << 4)) {
        uint8_t b = USART6->DT;
        if (b == 0xC8) crsf_pos = 0;
        if (crsf_pos < 64) crsf_buf[crsf_pos++] = b;

        if (crsf_pos >= 26 && crsf_buf[2] == 0x16) {
            uint8_t expected_len = crsf_buf[1];
            if (crsf_pos >= expected_len + 2) {
                uint8_t crc = crsf_crc8(&crsf_buf[2], expected_len - 1);
                if (crc == crsf_buf[2 + expected_len - 1]) {
                    const uint8_t *payload = &crsf_buf[3];

                    for (int ch = 0; ch < 16; ch++) {
                        channels[ch] = extract_11bit(payload, ch * 11);
                    }

                    crsf_data.r   = (int16_t)(channels[0] - 992);
                    crsf_data.p   = (int16_t)(channels[1] - 992);
                    crsf_data.thr = (int16_t)(channels[2] * 5 / 8 + 875);
                    crsf_data.y   = (int16_t)(channels[3] - 992);

                    uint16_t aux = 0;
                    for (int ch = 4; ch < 16; ch++) {
                        if (channels[ch] > 1500) {
                            aux |= (1 << (ch - 4));
                        }
                    }
                    crsf_data.aux = aux;

                    crsf_data.lq = 0xFF;
                    crsf_data.valid = true;
                    last_frame_ms = hal_ms();
                }
                crsf_pos = 0;
            }
        }
    }

    if (hal_ms() - last_frame_ms > 250) {
        crsf_data.lq = 0;
    }

    *out = crsf_data;
}
