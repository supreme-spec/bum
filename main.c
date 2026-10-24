#include "radun.h"
#include "radun_advanced.h"

imu_t imu;
crsf_t rx;
bat_t bat;
baro_t baro;
pid_t pid;
sp_t sp;
rpm_t rpm;
iterm_t iterm;
pg_t pg;
filt_t filt;
uint16_t motors[4];
bool armed = false, fs_active = false;
static uint32_t loop_cnt = 0, bb_cnt = 0;
float att[2] = {0};

void TIM6_DAC_IRQHandler(void) {
    TIM6->INTF = 0;

    /* 1. Gyro + filters */
    bmi270_process(&imu);
    filter_bank_apply(imu.dps);
    acc_filter_update(imu.acc, hal_ms());

    /* 2. Attitude (complementary filter) */
    imu_update_attitude(imu.dps, imu.acc, att, 1.0f/3200.0f);

    /* 3. Receiver */
    crsf_poll(&rx);
    failsafe(&rx, &fs_active);
    bool arming_allowed = arming_interlock_update(&rx, fs_active);
    if (fs_active && armed) {
        armed = false;
        pid.i[0] = pid.i[1] = pid.i[2] = 0;
        pid.out[0] = pid.out[1] = pid.out[2] = 0;
        flash_log_stop();
    }

    /* 4. Pre-arm safety checks */
    bool prearm_ok = prearm_check(&imu, &rx, &bat);

    /* 5. Arming — только если все проверки пройдены */
    if (!armed && arming_allowed && !fs_active && rx.valid && imu.healthy &&
        dshot_output_available() && !battery_low_voltage() && prearm_ok &&
        rx.thr < 1050 && rx.y > 700 && rx.r < -200 && rx.p < -200) {
        armed = true;
        pid.i[0] = pid.i[1] = pid.i[2] = 0;
        flash_log_start();
    }
    if (armed && (battery_low_voltage() || !prearm_ok || (rx.valid && rx.thr < 1050 && rx.y < -700))) {
        armed = false;
        flash_log_stop();
    }

    /* 5. Crash Recovery check (runs BEFORE PID) */
    bool is_crashed = false;
    if (armed) {
        is_crashed = crash_rec_check(imu.dps, rx.thr, armed);
        if (is_crashed) {
            crash_rec_reset_pid(pid.i);
            sp.sp[0] = 0; sp.sp[1] = 0; sp.sp[2] = 0;
        } else {
            modes_apply(&rx, att, sp.sp);
        }
    }

    /* 6. PID update */
    if (armed && !is_crashed) {
        pid_update(pid.kp, pid.ki, pid.kd, pid.kf, sp.sp, imu.dps, 1.0f/3200.0f,
                   (float)rx.thr, pid.out, pid.i, pid.gp, pid.dl);

        /* RLS autotune - adaptive gains based on response */
        for (int a = 0; a < 3; a++) {
            float rls_scale = rls_autotune_update(a, imu.dps[a], pid.out[a]);
            pid.out[a] *= rls_scale;
        }

        for (int a = 0; a < 3; a++) {
            pid.out[a] *= pg_scale(pg.scale, pg.axis, a);
            if (pid.out[a] > 1000.0f) pid.out[a] = 1000.0f;
            if (pid.out[a] < -1000.0f) pid.out[a] = -1000.0f;
        }
    } else {
        sp.sp[0] = sp.sp[1] = sp.sp[2] = 0;
        pid.out[0] = pid.out[1] = pid.out[2] = 0;
    }

    /* 9. Asymmetric mixer with blade-loss compensation */
    asymmetric_mixer(pid.out, (uint16_t)rx.thr, motors, blade_loss.efficiency);

    /* 10. DShot output */
    if (armed && !fs_active && dshot_output_available()) {
        dshot_send(motors);
    } else {
        uint16_t off[4] = {0, 0, 0, 0};
        dshot_send(off);
    }

    /* 11. Blackbox @800Hz */
    if (++bb_cnt >= 4) {
        bb_cnt = 0;
        flash_log(loop_cnt, &imu, &pid, motors, &bat);
    }

    loop_cnt++;
}

void TIM7_IRQHandler(void) {
    TIM7->INTF = 0;

    /* ADC voltage and current - read both channels */
    static uint8_t adc_channel = 0;
    static uint16_t vraw = 0, iraw = 0;

    ADC1->CTRL2 |= (1 << 22);
    while (ADC1->STS & (1 << 1));

    if (adc_channel == 0) {
        vraw = ADC1->RDAT;
        adc_channel = 1;
    } else {
        iraw = ADC1->RDAT;
        adc_channel = 0;
    }

    /* Process voltage */
    bat.v = ((float)vraw / 4095.0f) * 3.28f * VBAT_FACT;

    /* Process current from ADC channel 11 */
    /* Assuming 40mV/A current sensor with 3.3V reference */
    /* iraw / 4095 * 3.3V / 0.04V/A = current in A */
    float current_adc = ((float)iraw / 4095.0f) * 3.3f;
    bat.i = current_adc / 0.04f;  /* Convert to Amps */

    /* Fallback to model-based if ADC reading is invalid */
    float duty = (float)(rx.thr - 1000) / 1000.0f;
    if (duty < 0.0f) {
        duty = 0.0f;
    } else if (duty > 1.0f) {
        duty = 1.0f;
    }

    if (bat.i < 0.0f || bat.i > 100.0f) {
        bat.i = duty * 30.0f;
    }

    /* Battery management */
    battery_update(bat.v, bat.i);

    rpm_update(&rpm.model, &rpm.meas, &rpm.fused, &rpm.conf, rpm.hz, bat.v, duty);

    /* Update filter bank with RPM data */
    filter_bank_update_rpm(rpm.fused / 60.0f, rpm.conf, hal_ms());

    /* PropGuard update */
    if (armed) {
        propguard_update(bat.i, imu.dps, sp.sp, hal_ms());
    }

    IWDG->KR = 0xAAAA;
}

int main(void) {
    hal_init();

    spi_init(SPI1);
    spi_init(SPI2);

    gpio_af(GPIOB, GPIO_PIN_8|GPIO_PIN_9, 4);
    I2C1->CTRL2 = APB1_HZ / 1000000;
    I2C1->CTRL1 = (1<<15)|(1<<0);

    ADC1->SMPT1 = 0x0FFFFFFF;
    /* Channel 2: VBAT, Channel 11: Current sensor */
    ADC1->RSQ1 = (2 << 20) | (11 << 0);
    ADC1->RSQ2 = (12 << 5) | (13 << 10);
    ADC1->CTRL2 = (1<<8)|(1<<1)|(1<<22);

    bool sensor_configured = bmi270_config_upload() == 0;
    if (sensor_configured) {
        bmi270_real_init();
        bmi270_calibrate();
    }
    dshot_init_all();
    crsf_init();
    flash_log_init();
    usb_cdc_init();
    cli_init();

    pid_init(pid.kp, pid.ki, pid.kd, pid.kf, pid.out, pid.i, pid.gp, pid.dl);
    pg_init(&pg.ip, &pg.didt, &pg.dilp, &pg.thr, &pg.mismatch,
            &pg.until, &pg.cool, &pg.axis, &pg.scale, &pg.ev);
    filter_bank_init(FRAME_HZ, 3200.0f, 100.0f);
    rpm_init(&rpm.model, &rpm.meas, &rpm.fused, &rpm.conf, rpm.hz);
    iterm_init(iterm.i, 400.0f);

    /* Advanced safety init */
    crash_rec_init();
    blade_loss_init();
    cur_notch_init();
    propguard_init(9000.0f, 300.0f, 100);
    battery_init();
    rls_autotune_init();
    prearm_init();
    acc_filter_init();

    IWDG->KR = 0x5555; IWDG->PR = 4; IWDG->RLR = 1000;
    IWDG->KR = 0xAAAA; IWDG->KR = 0xCCCC;

    NVIC_SetPriority(54, 0);
    NVIC_SetPriority(55, 1);
    NVIC_ISER1 = (1 << 22) | (1 << 23);

    TIM6->PSC = (APB1_HZ / 3200) - 1; TIM6->CARR = 0;
    TIM6->CTRL1 = (1<<3)|(1<<0); TIM6->DINTEN = 1;
    TIM7->PSC = (APB1_HZ / 100) - 1; TIM7->CARR = 0;
    TIM7->CTRL1 = (1<<3)|(1<<0); TIM7->DINTEN = 1;

    delay_ms(2000);
    while (1) {
        IWDG->KR = 0xAAAA;
        usb_cdc_poll();
        
        /* Process CLI input from USART6 */
        uint8_t c;
        if (uart_getc(USART6, &c)) {
            cli_feed(c);
        }
        
        delay_ms(10);
    }
}