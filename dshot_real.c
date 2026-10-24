#include "radun.h"

static const uint16_t gcr_table[16] = {
    0x1E, 0x19, 0x18, 0x16, 0x15, 0x13, 0x12, 0x0E,
    0x0D, 0x0C, 0x0A, 0x09, 0x06, 0x05, 0x03, 0x1C
};

static uint16_t dshot_encode(uint16_t value, bool telem) {
    uint16_t c = value ^ (value >> 4) ^ (value >> 8);
    c &= 0x0F;
    uint16_t frame = (value << 4) | c;
    if (telem) frame |= 0x8000;
    uint16_t result = 0;
    for (int i = 0; i < 4; i++) {
        result <<= 5;
        result |= gcr_table[(frame >> (12 - i*4)) & 0x0F];
    }
    return result;
}

void dshot_init_all(void) {
    RCC->APB2 |= 1; RCC->APB1 |= (1 << 1);
    gpio_af(GPIOA, GPIO_PIN_8, 1); gpio_af(GPIOA, GPIO_PIN_9, 1);
    gpio_af(GPIOC, GPIO_PIN_8, 2); gpio_af(GPIOC, GPIO_PIN_9, 2);

    /* TIM1 CH1, CH2 → PWM1 mode (OCxM = 110), active-high, preload enabled */
    TIM1->PSC = 0; TIM1->CARR = 479;
    TIM1->CTRL1 = (1 << 7) | (1 << 0);
    TIM1->CHCTRL1 = (0x6 << 4) | (0x6 << 12) | (1 << 3) | (1 << 11);   /* OC1M=110, OC2M=110, preload */
    TIM1->CHCCTRL = (1 << 0) | (1 << 4);                                /* CC1E, CC2E */
    TIM1->BDCTRL = (1 << 15);                                           /* MOE */

    /* TIM3 CH3, CH4 → PWM1 mode (OCxM = 110), active-high, preload enabled */
    TIM3->PSC = 0; TIM3->CARR = 479;
    TIM3->CTRL1 = (1 << 7) | (1 << 0);
    TIM3->CHCTRL2 = (0x6 << 4) | (0x6 << 12) | (1 << 3) | (1 << 11);   /* OC3M=110, OC4M=110, preload */
    TIM3->CHCCTRL = (1 << 8) | (1 << 12);                               /* CC3E, CC4E */
    TIM3->BDCTRL = (1 << 15);                                           /* MOE */
}

void dshot_send(uint16_t motors[4]) {
    uint16_t ro[4] = {motors[1], motors[0], motors[2], motors[3]};
    for (int i = 0; i < 4; i++) {
        uint16_t pwm = ro[i];
        uint16_t dshot_val = 0;
        if (pwm > 1000) {
            dshot_val = (pwm - 1000) * 2 + 48;
            if (dshot_val > 2047) dshot_val = 2047;
        }
        uint16_t f = dshot_encode(dshot_val, false);
        switch (i) {
            case 0: TIM1->CH2C = f; break;
            case 1: TIM1->CH1C = f; break;
            case 2: TIM3->CH4C = f; break;
            case 3: TIM3->CH3C = f; break;
        }
    }
}