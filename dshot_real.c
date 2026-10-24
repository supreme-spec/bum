#include "radun.h"

/*
 * DShot600 для ACCIF435
 * MOTOR1: PA09 (TIM1 CH2)
 * MOTOR2: PA08 (TIM1 CH1)
 * MOTOR3: PC09 (TIM8 CH4)
 * MOTOR4: PC08 (TIM8 CH3)
 */

static bool dshot_ready = false;

void dshot_init_all(void) {
    RCC->APB2 |= (1 << 0) | (1 << 13);

    gpio_af(GPIOA, GPIO_PIN_8, 1);
    gpio_af(GPIOA, GPIO_PIN_9, 1);
    gpio_af(GPIOC, GPIO_PIN_8, 3);
    gpio_af(GPIOC, GPIO_PIN_9, 3);

    TIM1->PSC = 0;
    TIM1->CARR = 0xFFFF;
    TIM1->CTRL1 = 0;
    TIM1->CHCTRL1 = (6 << 4) | (1 << 3) | (6 << 12) | (1 << 11);
    TIM1->CHCTRL2 = (6 << 4) | (1 << 3) | (6 << 12) | (1 << 11);
    TIM1->CHCCTRL = (1 << 0) | (1 << 4) | (1 << 8) | (1 << 12);
    TIM1->BDCTRL  = (1 << 15);
    TIM1->CH1C = 0; TIM1->CH2C = 0;
    TIM1->CTRL1   = (1 << 0);

    TIM8->PSC = 0;
    TIM8->CARR = 0xFFFF;
    TIM8->CTRL1 = 0;
    TIM8->CHCTRL1 = (6 << 4) | (1 << 3) | (6 << 12) | (1 << 11);
    TIM8->CHCTRL2 = (6 << 4) | (1 << 3) | (6 << 12) | (1 << 11);
    TIM8->CHCCTRL = (1 << 0) | (1 << 4) | (1 << 8) | (1 << 12);
    TIM8->BDCTRL  = (1 << 15);
    TIM8->CH3C = 0; TIM8->CH4C = 0;
    TIM8->CTRL1   = (1 << 0);

    dshot_ready = true;
}

static inline uint16_t dshot_prepare_packet(uint16_t value, bool telemetry) {
    if (value > 2047) value = 2047;
    if (value < 48 && value > 0) value = 48;
    uint16_t packet = (value << 1) | (telemetry ? 1 : 0);
    uint16_t csum = packet ^ (packet >> 4) ^ (packet >> 8);
    csum &= 0x0F;
    packet = (packet << 4) | csum;
    return packet;
}

void dshot_send(uint16_t motors[4]) {
    if (!dshot_ready) {
        TIM1->CH1C = 0; TIM1->CH2C = 0;
        TIM8->CH3C = 0; TIM8->CH4C = 0;
        return;
    }

    for (int i = 0; i < 4; i++) {
        if (motors[i] < 48) {
        } else if (motors[i] > 2047) {
            motors[i] = 2047;
        }
    }
    (void)dshot_prepare_packet(motors[0], false);
    (void)dshot_prepare_packet(motors[1], false);
    (void)dshot_prepare_packet(motors[2], false);
    (void)dshot_prepare_packet(motors[3], false);

    TIM1->CH1C = 0;
    TIM1->CH2C = 0;
    TIM8->CH3C = 0;
    TIM8->CH4C = 0;
}

bool dshot_output_available(void) {
    return dshot_ready;
}

void dshot_telem_feed(int motor, uint32_t pulse_us) {
    (void)motor;
    (void)pulse_us;
}

float dshot_get_rpm(int motor) {
    (void)motor;
    return 0.0f;
}
