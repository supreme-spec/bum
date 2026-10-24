/* main_fixed.c — полная интеграция всех подсистем */
#include "radun.h"
#include "radun_advanced.h"
#include <math.h>
#include "watchdog.h"
#include "failsafe.h"
#include "dshot_bidir.h"
#include "rpm_filter.h"
#include "ubx.h"

void bmi270_process_fixed(imu_t *imu, uint32_t time_ms);
bool bmi270_is_healthy_fixed(void);
void filter_bank_init(float frame_hz, float fs, float slew_rate);
void filter_bank_apply(float dps[3]);
void filter_bank_reset(void);

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
static uint32_t last_health_check_ms = 0;
static uint32_t last_led_update_ms = 0;
static bool bmi_config_loaded;
static uint32_t last_rpm_feed_ms = 0;

void TIM6_DAC_IRQHandler(void) {
    uint32_t current_ms = hal_ms();
    TIM6->INTF = 0;

    bmi270_process_fixed(&imu, current_ms);
    filter_bank_apply(imu.dps);
    imu_update_attitude(imu.dps, imu.acc, imu.att, 1.0f/3200.0f);
    crsf_poll(&rx);

    failsafe(&rx, &fs_active);
    bool arming_allowed = arming_interlock_update(&rx, fs_active);
    if (fs_active && armed) {
        armed = false;
        pid.i[0] = pid.i[1] = pid.i[2] = 0;
        pid.out[0] = pid.out[1] = pid.out[2] = 0;
        flash_log_stop();
    }

    if (!armed && arming_allowed && !fs_active && bmi_config_loaded &&
        dshot_output_available() && rx.valid && imu.healthy &&
        rx.thr < 1050 && rx.y > 700 && rx.r < -200 && rx.p < -200) {
        armed = true;
        pid.i[0] = pid.i[1] = pid.i[2] = 0;
        filter_bank_reset();
        flash_log_start();
    }
    if (armed && rx.valid && rx.thr < 1050 && rx.y < -700) {
        armed = false;
        flash_log_stop();
    }

    bool is_crashed = false;
    if (armed) {
        is_crashed = crash_rec_check(imu.dps, rx.thr, armed);
        if (is_crashed) {
            crash_rec_reset_pid(pid.i);
            sp.sp[0] = sp.sp[1] = sp.sp[2] = 0;
        } else {
            modes_apply(&rx, imu.att, sp.sp);
        }
    }

    if (armed && !is_crashed) {
        float dt = 1.0f / 3200.0f;
        pid_update(pid.kp, pid.ki, pid.kd, pid.kf, sp.sp, imu.dps,
                   dt, (float)rx.thr, pid.out, pid.i, pid.gp, pid.dl);
        for (int axis = 0; axis < 3; axis++) {
            if (pid.out[axis] >  1000.0f) pid.out[axis] =  1000.0f;
            if (pid.out[axis] < -1000.0f) pid.out[axis] = -1000.0f;
        }
    } else {
        sp.sp[0] = sp.sp[1] = sp.sp[2] = 0;
        pid.out[0] = pid.out[1] = pid.out[2] = 0;
    }

    if (rpm_filter_healthy()) {
        float gyro_filt[3];
        gyro_filt[0] = rpm_filter_apply(0, imu.dps[0]);
        gyro_filt[1] = rpm_filter_apply(1, imu.dps[1]);
        gyro_filt[2] = rpm_filter_apply(2, imu.dps[2]);
        if (armed && !is_crashed) {
            float dt = 1.0f / 3200.0f;
            pid_update(pid.kp, pid.ki, pid.kd, pid.kf, sp.sp, gyro_filt,
                       dt, (float)rx.thr, pid.out, pid.i, pid.gp, pid.dl);
            for (int axis = 0; axis < 3; axis++) {
                if (pid.out[axis] >  1000.0f) pid.out[axis] =  1000.0f;
                if (pid.out[axis] < -1000.0f) pid.out[axis] = -1000.0f;
            }
        }
    }

    asymmetric_mixer(pid.out, (uint16_t)rx.thr, motors, blade_loss.efficiency);

    if (armed && !fs_active && bmi_config_loaded && imu.healthy &&
        dshot_output_available()) {
        dshot_send(motors);
    } else {
        uint16_t off[4] = {0, 0, 0, 0};
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
    bat.v = ((float)vraw / 4095.0f) * 3.28f * VBAT_FACT;

    if (current_ms - last_health_check_ms >= 1000U) {
        last_health_check_ms = current_ms;
        if (!bmi270_is_healthy_fixed()) {
            armed = false;
            fs_active = true;
            pid.i[0] = pid.i[1] = pid.i[2] = 0;
            pid.out[0] = pid.out[1] = pid.out[2] = 0;
            uint16_t off[4] = {0, 0, 0, 0};
            dshot_send(off);
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

    spi_init(SPI1);
    spi_init(SPI2);
    gpio_af(GPIOB, GPIO_PIN_8 | GPIO_PIN_9, 4);
    I2C1->CTRL2 = APB1_HZ / 1000000;
    I2C1->CTRL1 = (1 << 15) | (1 << 0);

    ADC1->SMPT1 = 0x0FFFFFFF;
    ADC1->RSQ1  = (2 << 20);
    ADC1->RSQ2  = (11 << 0) | (12 << 5) | (13 << 10);
    ADC1->CTRL2 = (1 << 8) | (1 << 1) | (1 << 22);

    bmi_config_loaded = bmi270_config_upload() == 0;
    if (bmi_config_loaded) {
        bmi270_real_init();
        bmi_config_loaded = bmi270_calibrate_fixed();
    }
    dshot_init_all();
    crsf_init();
    flash_log_init();

    pid_init(pid.kp, pid.ki, pid.kd, pid.kf, pid.out, pid.i, pid.gp, pid.dl);
    filter_bank_init(FRAME_HZ, 3200.0f, 200.0f);
    crash_rec_init();
    blade_loss_init();

    IWDG->KR = 0x5555;
    IWDG->PR = 4;
    IWDG->RLR = 1000;
    IWDG->KR = 0xAAAA;
    IWDG->KR = 0xCCCC;

    watchdog_init(200);
    failsafe_init();
    dshot_bidir_init();
    rpm_filter_init();
    ubx_init();

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
        dshot_bidir_process_telemetry();
        uint32_t now_ms = hal_ms();
        if (now_ms - last_rpm_feed_ms >= 2) {
            last_rpm_feed_ms = now_ms;
            for (int m = 0; m < 4; m++) {
                const esc_telemetry_t *t = dshot_bidir_get_telemetry(m);
                if (t && t->valid && t->erpm > 0) {
                    rpm_filter_feed_erpm(m, t->erpm);
                }
            }
            dshot_bidir_request_telemetry_all();
        }
        rpm_filter_update_coeffs();
        ubx_process();
        failsafe_update_gps_from_ubx();
        watchdog_feed();
        __asm__ volatile ("nop");
    }
}
