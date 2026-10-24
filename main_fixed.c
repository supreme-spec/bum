/* main_fixed.c — полная интеграция всех подсистем */
#include "radun.h"
#include "radun_rls_tuning.h"
#include "radun_advanced.h"
#include <math.h>

void bmi270_process_fixed(imu_t *imu, uint32_t time_ms);
bool bmi270_is_healthy_fixed(void);
void bmi270_calibrate_fixed(void);
void bmi270_reset_health(void);
bool bmi270_recal_pending(void);
void bmi270_do_recal(void);

void filter_bank_init(float frame_hz, float fs, float slew_rate);
void filter_bank_update_rpm(float rpm_hz, float rpm_confidence, uint32_t time_ms);
void filter_bank_apply(float dps[3]);
void filter_bank_reset(void);
void filter_bank_get_frequencies(float fc[3][3]);

void propguard_init(float threshold, float mismatch, uint32_t cooldown_ms);
void propguard_update(float current, float gyro[3], float sp[3], uint32_t time_ms);
float propguard_get_scale(int axis);
void propguard_reset(void);

void flash_log_process(void);

imu_t   imu;
crsf_t  rx;
bat_t   bat;
baro_t  baro;
pid_t   pid;
sp_t    sp;
rpm_t   rpm;
iterm_t iterm;
uint16_t motors[4];
bool armed = false, fs_active = false;

static uint32_t loop_cnt = 0, bb_cnt = 0;
static uint32_t last_rpm_update_ms = 0;
static uint32_t last_health_check_ms = 0;
static uint32_t boot_time_ms = 0;
static uint32_t last_led_update_ms = 0;

static float estimated_current = 0.0f;

float filter_freqs[3][3];

void TIM6_DAC_IRQHandler(void) {
    uint32_t current_ms = hal_ms();
    TIM6->INTF = 0;

    bmi270_process_fixed(&imu, current_ms);
    filter_bank_apply(imu.dps);
    imu_update_attitude(imu.dps, imu.acc, imu.att, 1.0f/3200.0f);
    crsf_poll(&rx);

    if (!armed && imu.healthy && rx.thr < 1050 && rx.y > 700 && rx.r < -200 && rx.p < -200) {
        armed = true;
        pid.i[0] = pid.i[1] = pid.i[2] = 0;
        filter_bank_reset();
        propguard_reset();
        flash_log_start();
    }
    if (armed && rx.thr < 1050 && rx.y < -700) {
        armed = false;
        flash_log_stop();
    }

    bool is_crashed = false;
    if (armed) {
        if (nn_crash_detect(imu.dps)) {
            is_crashed = true;
        } else {
            is_crashed = crash_rec_check(imu.dps, rx.thr, armed);
        }
        if (is_crashed) {
            crash_rec_reset_pid(pid.i);
            sp.sp[0] = sp.sp[1] = sp.sp[2] = 0;
        } else {
            modes_apply(&rx, imu.att, sp.sp);
        }
    }

    propguard_update(estimated_current, imu.dps, sp.sp, current_ms);

    float base_kp[3] = {35, 37, 32};
    float base_ki[3] = {63, 53, 57};
    for (int a = 0; a < 3; a++) {
        float scale = rls_autotune_update(a, imu.dps[a], pid.out[a]);
        if (scale < 0.5f) scale = 0.5f;
        if (scale > 2.0f) scale = 2.0f;
        pid.kp[a] = base_kp[a] * scale;
        pid.ki[a] = base_ki[a] * scale;
    }

    float ff_pred[3] = {0, 0, 0};
    if (armed) {
        pred_ff_update(sp.sp, ff_pred, 1.0f/3200.0f);
    }

    if (armed && !is_crashed) {
        float dt = 1.0f / 3200.0f;
        pid_update(pid.kp, pid.ki, pid.kd, pid.kf, sp.sp, imu.dps, dt, (float)rx.thr, pid.out, pid.i, pid.gp, pid.dl);
        float ff_pred[3];
        pred_ff_update(sp.sp, ff_pred, dt);
        for (int axis = 0; axis < 3; axis++) {
            pid.out[axis] += ff_pred[axis];
            pid.out[axis] *= propguard_get_scale(axis);
            if (pid.out[axis] >  1000.0f) pid.out[axis] =  1000.0f;
            if (pid.out[axis] < -1000.0f) pid.out[axis] = -1000.0f;
        }
    } else {
        sp.sp[0] = sp.sp[1] = sp.sp[2] = 0;
        pid.out[0] = pid.out[1] = pid.out[2] = 0;
    }

    failsafe(&rx, &fs_active);

    asymmetric_mixer(pid.out, (uint16_t)rx.thr, motors, blade_loss.efficiency);

    if (armed && !fs_active && imu.healthy) {
        dshot_send(motors);
    } else {
        uint16_t off[4] = {1000, 1000, 1000, 1000};
        dshot_send(off);
    }

    if (++bb_cnt >= 4) {
        bb_cnt = 0;
        flash_log(loop_cnt, &imu, &pid, motors, &bat);
    }

    loop_cnt++;
}

void TIM7_IRQHandler(void) {
    uint32_t current_ms = hal_ms();
    TIM7->INTF = 0;

    ADC1->CTRL2 |= (1 << 22);
    while (ADC1->STS & (1 << 1));
    uint16_t vraw = ADC1->RDAT;
    bat.v = ((float)vraw / 4095.0f) * 3.28f * 8.5f;

    float duty = (float)(rx.thr - 1000) / 1000.0f;
    if (duty < 0) duty = 0;
    float i_est = duty * 20.0f;
    estimated_current = 0.95f * estimated_current + 0.05f * i_est;

    rpm_update(&rpm.model, &rpm.meas, &rpm.fused, &rpm.conf, rpm.hz, bat.v, duty);

    if (current_ms - last_rpm_update_ms >= 100U) {
        last_rpm_update_ms = current_ms;
        filter_bank_update_rpm(rpm.hz[0], rpm.conf, current_ms);
    }

    float currents[4] = {
        estimated_current, estimated_current,
        estimated_current, estimated_current
    };
    blade_loss_detect(currents, imu.dps);

    float notch_hz = cur_notch_update(duty, bat.v, 14);
    (void)notch_hz;

    if (current_ms - last_health_check_ms >= 1000U) {
        last_health_check_ms = current_ms;
        if (!bmi270_is_healthy_fixed()) {
            bmi270_reset_health();
            if (!imu.healthy) {
                armed = false;
        uint16_t off[4] = {0, 0, 0, 0};
                dshot_send(off);
            }
        }
    }

    IWDG->KR = 0xAAAA;

    if (current_ms - last_led_update_ms >= 500U) {
        last_led_update_ms = current_ms;
        if (!imu.healthy) {
            static bool led_toggle = false;
            led_toggle = !led_toggle;
            if (led_toggle) gpio_set(GPIOC, GPIO_PIN_14);
            else            gpio_clr(GPIOC, GPIO_PIN_14);
        } else if (armed) {
            gpio_set(GPIOC, GPIO_PIN_14);
        } else {
            static uint32_t slow_blink = 0;
            if (++slow_blink & 1) gpio_set(GPIOC, GPIO_PIN_14);
            else                  gpio_clr(GPIOC, GPIO_PIN_14);
        }
    }
}

int main(void) {
    hal_init();
    boot_time_ms = hal_ms();

    spi_init(SPI1);
    spi_init(SPI2);
    gpio_af(GPIOB, GPIO_PIN_8 | GPIO_PIN_9, 4);
    I2C1->CTRL2 = APB1_HZ / 1000000;
    I2C1->CTRL1 = (1 << 15) | (1 << 0);

    ADC1->SMPT1 = 0x0FFFFFFF;
    ADC1->RSQ1  = (2 << 20);
    ADC1->RSQ2  = (11 << 0) | (12 << 5) | (13 << 10);
    ADC1->CTRL2 = (1 << 8) | (1 << 1) | (1 << 22);

    if (bmi270_config_upload() != 0) {
    }
    bmi270_real_init();
    bmi270_calibrate_fixed();
    dshot_init_all();
    crsf_init();
    flash_log_init();

    pid_init(pid.kp, pid.ki, pid.kd, pid.kf, pid.out, pid.i, pid.gp, pid.dl);
    filter_bank_init(FRAME_HZ, 3200.0f, 200.0f);
    propguard_init(9000.0f, 300.0f, 100);
    rpm_init(&rpm.model, &rpm.meas, &rpm.fused, &rpm.conf, rpm.hz);
    iterm_init(iterm.i, 400.0f);

    pred_ff_init(pid.kf);
    crash_rec_init();
    blade_loss_init();
    cur_notch_init();
    rls_autotune_init();

    IWDG->KR = 0x5555;
    IWDG->PR = 4;
    IWDG->RLR = 1000;
    IWDG->KR = 0xAAAA;
    IWDG->KR = 0xCCCC;

    NVIC_SetPriority(54, 0);
    NVIC_SetPriority(55, 1);
    NVIC_ISER1 = (1 << 22) | (1 << 23);

    TIM6->PSC = (APB1_HZ / 3200) - 1;
    TIM6->CARR = 0;
    TIM6->CTRL1 = (1 << 2) | (1 << 0);
    TIM6->DINTEN = 1;

    TIM7->PSC = (APB1_HZ / 800) - 1;
    TIM7->CARR = 0;
    TIM7->CTRL1 = (1 << 2) | (1 << 0);
    TIM7->DINTEN = 1;

    for (int i = 0; i < 3; i++) {
        gpio_set(GPIOC, GPIO_PIN_14);
        delay_ms(100);
        gpio_clr(GPIOC, GPIO_PIN_14);
        delay_ms(100);
    }

    while (1) {
        flash_log_process();

        if (!armed && bmi270_recal_pending()) {
            bmi270_do_recal();
        }

        IWDG->KR = 0xAAAA;
        asm volatile("wfi");
    }
}
