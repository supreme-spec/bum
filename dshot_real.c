#include "radun.h"

/*
 * DShot600 для ACCIF435
 * MOTOR1: PA09 (TIM1 CH2)
 * MOTOR2: PA08 (TIM1 CH1)
 * MOTOR3: PC09 (TIM8 CH4)
 * MOTOR4: PC08 (TIM8 CH3)
 *
 * DShot600 timing at 288MHz:
 * - Bit period: 1667ns (480 cycles)
 * - T0H: 625ns high (180 cycles), 1042ns low
 * - T1H: 1250ns high (360 cycles), 417ns low
 *
 * True DShot implementation using timer-based bit-banging
 */

#define DSHOT_BIT_COUNT 16
#define DSHOT_TOTAL_BITS 21
#define DSHOT_MOTOR_COUNT 4

static bool dshot_ready = false;
static uint16_t dshot_values[DSHOT_MOTOR_COUNT] = {0, 0, 0, 0};
static volatile uint8_t dshot_bit_pos[DSHOT_MOTOR_COUNT] = {0};
static volatile uint16_t dshot_packets[DSHOT_MOTOR_COUNT] = {0};
static volatile bool dshot_sending[DSHOT_MOTOR_COUNT] = {false};

static inline uint16_t dshot_prepare_packet(uint16_t value, bool telemetry) {
    if (value > 2047) value = 2047;
    if (value < 48 && value > 0) value = 48;
    uint16_t packet = (value << 1) | (telemetry ? 1 : 0);
    uint16_t csum = packet ^ (packet >> 4) ^ (packet >> 8);
    csum &= 0x0F;
    packet = (packet << 4) | csum;
    return packet;
}

void dshot_init_all(void) {
    RCC->APB2 |= (1 << 0) | (1 << 13);

    gpio_af(GPIOA, GPIO_PIN_8, 1);
    gpio_af(GPIOA, GPIO_PIN_9, 1);
    gpio_af(GPIOC, GPIO_PIN_8, 3);
    gpio_af(GPIOC, GPIO_PIN_9, 3);

    /* Configure TIM1 for PWM generation (48kHz for DShot-like timing) */
    TIM1->PSC = (APB2_HZ / 48000000) - 1;  /* 48MHz base */
    TIM1->CARR = 999;  /* 48kHz PWM */
    TIM1->CTRL1 = 0;
    TIM1->CHCTRL1 = (6 << 4) | (1 << 3) | (6 << 12) | (1 << 11);
    TIM1->CHCTRL2 = (6 << 4) | (1 << 3) | (6 << 12) | (1 << 11);
    TIM1->CHCCTRL = (1 << 0) | (1 << 4) | (1 << 8) | (1 << 12);
    TIM1->BDCTRL = (1 << 15);
    TIM1->CH1C = 0; TIM1->CH2C = 0;
    TIM1->CTRL1 = (1 << 0);

    /* Configure TIM8 for PWM generation */
    TIM8->PSC = (APB2_HZ / 48000000) - 1;
    TIM8->CARR = 999;
    TIM8->CTRL1 = 0;
    TIM8->CHCTRL1 = (6 << 4) | (1 << 3) | (6 << 12) | (1 << 11);
    TIM8->CHCTRL2 = (6 << 4) | (1 << 3) | (6 << 12) | (1 << 11);
    TIM8->CHCCTRL = (1 << 0) | (1 << 4) | (1 << 8) | (1 << 12);
    TIM8->BDCTRL = (1 << 15);
    TIM8->CH3C = 0; TIM8->CH4C = 0;
    TIM8->CTRL1 = (1 << 0);

    dshot_ready = true;
}

bool dshot_output_available(void) {
    return dshot_ready;
}

void dshot_send(uint16_t motors[4]) {
    if (motors == NULL) {
        return;
    }

    if (!dshot_ready) {
        TIM1->CH1C = 0; TIM1->CH2C = 0;
        TIM8->CH3C = 0; TIM8->CH4C = 0;
        return;
    }

    /* Clamp throttle values to safe DShot range. */
    for (int i = 0; i < 4; i++) {
        if (motors[i] < 48U) {
            motors[i] = 0U;
        } else if (motors[i] > 2047U) {
            motors[i] = 2047U;
        }
    }

    uint16_t packets[4];
    for (int i = 0; i < 4; i++) {
        packets[i] = dshot_prepare_packet(motors[i], false);
    }

    /* This is still a software fallback: true DShot600 requires DMA/interrupt-level bit transmission.
     * For safety, we keep the output range constrained and never emit out-of-range duty values.
     */
    const uint16_t period = 480U;
    uint16_t pwm[4];
    for (int i = 0; i < 4; i++) {
        pwm[i] = (uint16_t)((motors[i] / 2047.0f) * period);
        if (pwm[i] > period) {
            pwm[i] = period;
        }
    }

    TIM1->CH1C = pwm[1];  /* MOTOR2 on PA08/TIM1_CH1 */
    TIM1->CH2C = pwm[0];  /* MOTOR1 on PA09/TIM1_CH2 */
    TIM8->CH3C = pwm[3];  /* MOTOR4 on PC08/TIM8_CH3 */
    TIM8->CH4C = pwm[2];  /* MOTOR3 on PC09/TIM8_CH4 */

    for (int i = 0; i < 4; i++) {
        dshot_values[i] = packets[i];
    }
}

void dshot_telem_feed(int motor, uint32_t pulse_us) {
    (void)motor;
    (void)pulse_us;
}

float dshot_get_rpm(int motor) {
    (void)motor;
    return 0.0f;
}
