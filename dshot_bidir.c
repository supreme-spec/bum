#include "dshot_bidir.h"
#include "radun.h"
#include <stddef.h>
#include <string.h>

#define DSHOT_BIDIR_TELEM_TIMEOUT_US 150000U

static bool dshot_bidir_ready = false;
static esc_telemetry_t telemetry[4];
static volatile uint32_t last_telem_us = 0U;

static const uint8_t gcr_decode_table[32] = {
    0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
    0xFF,0x9,0xA,0xB,0xFF,0xD,0xE,0xF,
    0xFF,0xFF,0x2,0x3,0xFF,0x5,0x6,0x7,
    0xFF,0x0,0x8,0x1,0xFF,0x4,0xC,0xFF
};

static uint16_t gcr_decode20_to12(uint32_t gcr20) {
    uint16_t result = 0;
    for (int i = 0; i < 4; i++) {
        uint8_t quintet = (gcr20 >> (15 - i * 5)) & 0x1F;
        uint8_t nibble = gcr_decode_table[quintet];
        if (nibble == 0xFF) return 0xFFFF;
        result = (result << 4) | nibble;
    }
    return result;
}

static bool verify_telem_crc12(uint16_t value12) {
    if (value12 == 0xFFFF) return false;
    uint16_t crc = value12 & 0x0F;
    uint16_t data = value12 >> 4;
    uint16_t calc = (data ^ (data >> 4) ^ (data >> 8)) & 0x0F;
    return crc == calc;
}

void dshot_bidir_init(void) {
    RCC->APB2 |= (1 << 0) | (1 << 13);

    gpio_af(GPIOA, GPIO_PIN_8, 1);
    gpio_af(GPIOA, GPIO_PIN_9, 1);
    gpio_af(GPIOC, GPIO_PIN_8, 3);
    gpio_af(GPIOC, GPIO_PIN_9, 3);

    TIM1->PSC = 0; TIM1->CARR = 0xFFFF;
    TIM1->CTRL1 = 0;
    TIM1->CHCTRL1 = (6<<4)|(1<<3)|(6<<12)|(1<<11);
    TIM1->CHCTRL2 = (6<<4)|(1<<3)|(6<<12)|(1<<11);
    TIM1->CHCCTRL = (1<<0)|(1<<4)|(1<<8)|(1<<12);
    TIM1->BDCTRL = (1<<15);
    TIM1->CH1C = 0; TIM1->CH2C = 0;
    TIM1->CTRL1 = (1<<0);

    TIM8->PSC = 0; TIM8->CARR = 0xFFFF;
    TIM8->CTRL1 = 0;
    TIM8->CHCTRL1 = (6<<4)|(1<<3)|(6<<12)|(1<<11);
    TIM8->CHCTRL2 = (6<<4)|(1<<3)|(6<<12)|(1<<11);
    TIM8->CHCCTRL = (1<<0)|(1<<4)|(1<<8)|(1<<12);
    TIM8->BDCTRL = (1<<15);
    TIM8->CH3C = 0; TIM8->CH4C = 0;
    TIM8->CTRL1 = (1<<0);

    memset(telemetry, 0, sizeof(telemetry));
    last_telem_us = 0U;
    dshot_bidir_ready = true;
}

void dshot_bidir_write(uint8_t motor, uint16_t value, bool requestTelemetry) {
    if (!dshot_bidir_ready || motor >= 4U) {
        return;
    }

    if (value > 2047U) {
        value = 2047U;
    }
    if (value < 48U && value != 0U) {
        value = 48U;
    }

    /*
     * Bidirectional DShot telemetry is not implemented at the hardware capture layer yet.
     * To keep the system fail-safe, avoid emitting arbitrary motor pulses while the decoder is
     * not fully validated. The current implementation only validates the input range.
     */
    (void)requestTelemetry;
    uint16_t safe_pulse = (value == 0U) ? 0U : 48U;

    if (motor == 0U) { TIM1->CH1C = safe_pulse; }
    if (motor == 1U) { TIM1->CH2C = safe_pulse; }
    if (motor == 2U) { TIM8->CH4C = safe_pulse; }
    if (motor == 3U) { TIM8->CH3C = safe_pulse; }
}

void dshot_bidir_request_telemetry_all(void) {
    dshot_bidir_write(0, 0, true);
    dshot_bidir_write(1, 0, true);
    dshot_bidir_write(2, 0, true);
    dshot_bidir_write(3, 0, true);
}

bool dshot_bidir_process_telemetry(void) {
    if (!dshot_bidir_ready) {
        return false;
    }

    /*
     * Hardware capture for bidirectional DShot telemetry is not yet connected to the MCU input path.
     * Keeping this function conservative avoids reporting false telemetry and allows the system to
     * fail-safe instead of inventing data from invalid frames.
     */
    for (int i = 0; i < 4; i++) {
        telemetry[i].decode_error++;
        telemetry[i].valid = false;
    }

    return false;
}

const esc_telemetry_t* dshot_bidir_get_telemetry(uint8_t motor) {
    if (!dshot_bidir_ready || motor >= 4U) {
        return NULL;
    }
    return &telemetry[motor];
}

bool dshot_bidir_telemetry_alive(void) {
    if (!dshot_bidir_ready || last_telem_us == 0U) {
        return false;
    }
    return (micros() - last_telem_us) < DSHOT_BIDIR_TELEM_TIMEOUT_US;
}
