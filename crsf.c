/* crsf.c - CRSF receiver parser for the 16-channel RC frame */
#include "radun.h"

#define CRSF_BUFFER_SIZE 64
#define CRSF_RC_FRAME_LENGTH 24
#define CRSF_RC_CHANNELS_TYPE 0x16
#define CRSF_FRAME_TIMEOUT_MS 100U

static uint8_t crsf_buf[CRSF_BUFFER_SIZE];
static uint8_t crsf_pos;
static uint8_t crsf_frame_size;
static crsf_t crsf_data;
static uint32_t last_frame_ms;
static uint16_t channels[16];

static uint8_t crsf_crc8(const uint8_t *data, uint8_t len) {
    uint8_t crc = 0;
    for (uint8_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int j = 0; j < 8; j++) {
            crc = (crc & 0x80U) ? (uint8_t)((crc << 1) ^ 0xD5U)
                                : (uint8_t)(crc << 1);
        }
    }
    return crc;
}

static uint16_t extract_11bit(const uint8_t *payload, uint8_t bit_offset) {
    uint8_t byte_offset = bit_offset / 8U;
    uint8_t bit_in_byte = bit_offset % 8U;
    uint32_t raw = (uint32_t)payload[byte_offset] |
                   ((uint32_t)payload[byte_offset + 1U] << 8);
    if (bit_in_byte > 5U) {
        raw |= (uint32_t)payload[byte_offset + 2U] << 16;
    }
    return (uint16_t)((raw >> bit_in_byte) & 0x7FFU);
}

static void process_rc_frame(void) {
    if (crsf_buf[1] != CRSF_RC_FRAME_LENGTH ||
        crsf_buf[2] != CRSF_RC_CHANNELS_TYPE ||
        crsf_crc8(&crsf_buf[2], CRSF_RC_FRAME_LENGTH - 1U) !=
            crsf_buf[CRSF_RC_FRAME_LENGTH + 1U]) {
        return;
    }

    const uint8_t *payload = &crsf_buf[3];
    for (uint8_t ch = 0; ch < 16; ch++) {
        channels[ch] = extract_11bit(payload, (uint8_t)(ch * 11U));
    }

    crsf_data.r = (int16_t)channels[0] - 992;
    crsf_data.p = (int16_t)channels[1] - 992;
    crsf_data.thr = (int16_t)(channels[2] * 5U / 8U + 875U);
    crsf_data.y = (int16_t)channels[3] - 992;

    uint16_t aux = 0;
    for (uint8_t ch = 4; ch < 16; ch++) {
        if (channels[ch] > 1500U) {
            aux |= (uint16_t)(1U << (ch - 4U));
        }
    }
    crsf_data.aux = aux;
    crsf_data.lq = 0xFF;
    crsf_data.valid = true;
    last_frame_ms = hal_ms();
}

void crsf_init(void) {
    crsf_pos = 0;
    crsf_frame_size = 0;
    crsf_data = (crsf_t){0};
    channels[0] = channels[1] = channels[2] = channels[3] = 992U;

    RCC->APB2 |= (1 << 5);
    gpio_af(GPIOC, GPIO_PIN_7, 8);
    gpio_af(GPIOC, GPIO_PIN_6, 8);
    uart_init(USART6, 420000);
}

void crsf_poll(crsf_t *out) {
    while (USART6->STS & (1 << 4)) {
        uint8_t byte = (uint8_t)USART6->DT;

        if (crsf_pos == 0) {
            if (byte != 0xC8U) {
                continue;
            }
            crsf_buf[crsf_pos++] = byte;
            continue;
        }

        if (crsf_pos >= CRSF_BUFFER_SIZE) {
            crsf_pos = 0;
            crsf_frame_size = 0;
            continue;
        }
        crsf_buf[crsf_pos++] = byte;

        if (crsf_pos == 2U) {
            uint8_t frame_length = crsf_buf[1];
            if (frame_length < 2U || frame_length > CRSF_BUFFER_SIZE - 2U) {
                crsf_pos = 0;
                crsf_frame_size = 0;
                continue;
            }
            crsf_frame_size = (uint8_t)(frame_length + 2U);
        }

        if (crsf_frame_size != 0U && crsf_pos == crsf_frame_size) {
            process_rc_frame();
            crsf_pos = 0;
            crsf_frame_size = 0;
        }
    }

    if (crsf_data.valid &&
        (uint32_t)(hal_ms() - last_frame_ms) > CRSF_FRAME_TIMEOUT_MS) {
        crsf_data.valid = false;
        crsf_data.lq = 0;
        crsf_data.r = 0;
        crsf_data.p = 0;
        crsf_data.y = 0;
        crsf_data.thr = 1000;
        crsf_data.aux = 0;
    }

    *out = crsf_data;
}
