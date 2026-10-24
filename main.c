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
    filt_apply(filt.n, imu.dps);

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

    /* 4. Arming — только если IMU здоров */
    if (!armed && arming_allowed && !fs_active && rx.valid && imu.healthy &&
        dshot_output_available() &&
        rx.thr < 1050 && rx.y > 700 && rx.r < -200 && rx.p < -200) {
        armed = true;
        pid.i[0] = pid.i[1] = pid.i[2] = 0;
        flash_log_start();
    }
    if (armed && rx.valid && rx.thr < 1050 && rx.y < -700) {
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

    /* ADC voltage */
    ADC1->CTRL2 |= (1 << 22);
    while (ADC1->STS & (1 << 1));
    uint16_t vraw = ADC1->RDAT;
    bat.v = ((float)vraw / 4095.0f) * 3.28f * VBAT_FACT;

    float duty = (float)(rx.thr - 1000) / 1000.0f;

    rpm_update(&rpm.model, &rpm.meas, &rpm.fused, &rpm.conf, rpm.hz, bat.v, duty);

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
    ADC1->RSQ1 = (2 << 20);
    ADC1->RSQ2 = (11 << 0) | (12 << 5) | (13 << 10);
    ADC1->CTRL2 = (1<<8)|(1<<1)|(1<<22);

    bool sensor_configured = bmi270_config_upload() == 0;
    if (sensor_configured) {
        bmi270_real_init();
        bmi270_calibrate();
    }
    dshot_init_all();
    crsf_init();
    flash_log_init();

    pid_init(pid.kp, pid.ki, pid.kd, pid.kf, pid.out, pid.i, pid.gp, pid.dl);
    pg_init(&pg.ip, &pg.didt, &pg.dilp, &pg.thr, &pg.mismatch,
            &pg.until, &pg.cool, &pg.axis, &pg.scale, &pg.ev);
    filt_init(filt.n, 3200.0f, FRAME_HZ);
    rpm_init(&rpm.model, &rpm.meas, &rpm.fused, &rpm.conf, rpm.hz);
    iterm_init(iterm.i, 400.0f);

    /* Advanced safety init */
    crash_rec_init();
    blade_loss_init();
    cur_notch_init();

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
        delay_ms(10);
    }
}