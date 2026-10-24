#include "radun.h"
#include "radun_advanced.h"
#include "radun_rls_tuning.h"

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
static uint32_t notch_last_ms = 0;
float att[2] = {0};
float base_kp[3], base_ki[3], base_kd[3];
float rls_scale[3] = {1.0f, 1.0f, 1.0f};

void TIM6_DAC_IRQHandler(void) {
    TIM6->INTF = 0;

    /* 1. Gyro + filters */
    bmi270_process(&imu);
    filt_apply(filt.n, imu.dps);

    /* 2. Attitude (complementary filter) */
    imu_update_attitude(imu.dps, imu.acc, att, 1.0f/3200.0f);

    /* 3. Receiver */
    crsf_poll(&rx);

    /* 4. Arming — только если IMU здоров */
    if (!armed && imu.healthy && rx.thr < 1050 && rx.y > 700 && rx.r < -200 && rx.p < -200) {
        armed = true;
        pid.i[0] = pid.i[1] = pid.i[2] = 0;
        flash_log_start();
    }
    if (armed && rx.thr < 1050 && rx.y < -700) {
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

    /* 6. RLS autotuning с жёсткими границами */
    for (int a = 0; a < 3; a++) {
        float scale = rls_autotune_update(a, imu.dps[a], pid.out[a]);
        rls_scale[a] = 0.999f * rls_scale[a] + 0.001f * scale;
        if (rls_scale[a] < 0.5f) rls_scale[a] = 0.5f;
        if (rls_scale[a] > 2.0f) rls_scale[a] = 2.0f;
        pid.kp[a] = base_kp[a] * rls_scale[a];
        pid.ki[a] = base_ki[a] * rls_scale[a];
    }

    /* 7. Predictive Feed-Forward by Jerk */
    float ff_pred[3];
    pred_ff_update(sp.sp, ff_pred, 1.0f/3200.0f);

    /* 8. PID update (with TPA already inside pid_update) */
    if (armed && !is_crashed) {
        pid_update(pid.kp, pid.ki, pid.kd, pid.kf, sp.sp, imu.dps, 1.0f/3200.0f,
                   (float)rx.thr, pid.out, pid.i, pid.gp, pid.dl);
        /* Add predictive FF on top of PID */
        for (int a = 0; a < 3; a++) {
            pid.out[a] += ff_pred[a];
            pid.out[a] *= pg_scale(pg.scale, pg.axis, a);
            if (pid.out[a] > 1000.0f) pid.out[a] = 1000.0f;
            if (pid.out[a] < -1000.0f) pid.out[a] = -1000.0f;
        }
    } else {
        sp.sp[0] = sp.sp[1] = sp.sp[2] = 0;
        pid.out[0] = pid.out[1] = pid.out[2] = 0;
    }

    /* 8. Failsafe */
    failsafe(&rx, &fs_active);

    /* 9. Asymmetric mixer with blade-loss compensation */
    asymmetric_mixer(pid.out, (uint16_t)rx.thr, motors, blade_loss.efficiency);

    /* 10. DShot output */
    if (armed && !fs_active) {
        dshot_send(motors);
    } else {
        uint16_t off[4] = {48, 48, 48, 48};
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
    bat.v = ((float)vraw / 4095.0f) * 3.28f * 8.5f;

    /* Current notch predictor from ESC telemetry (duty-based model) */
    float duty = (float)(rx.thr - 1000) / 1000.0f;
    float notch_hz = cur_notch_update(duty, bat.v, 14);
    if (notch_hz > 1.0f && notch_hz < 1400.0f) {
        /* Rate limit: update notch coefficients max once per 100ms */
        if (hal_ms() - notch_last_ms >= 100U) {
            notch_last_ms = hal_ms();
            for (int a = 0; a < 2; a++) {
                filt_notch_update(filt.n, a, 1, notch_hz, 4.0f, 3200.0f);
            }
        }
    }

    rpm_update(&rpm.model, &rpm.meas, &rpm.fused, &rpm.conf, rpm.hz, bat.v, duty);

    /* Blade-loss detector */
    ADC1->CTRL2 |= (1 << 22);
    while (ADC1->STS & (1 << 1));
    float currents[4] = {
        (float)ADC1->RDAT / 4095.0f * 5.0f,
        (float)ADC1->RDAT / 4095.0f * 5.0f,
        (float)ADC1->RDAT / 4095.0f * 5.0f,
        (float)ADC1->RDAT / 4095.0f * 5.0f
    };
    blade_loss_detect(currents, imu.dps);

    /* Blade-loss auto-recovery: deactivate if currents normalize for 2 seconds */
    if (blade_loss.active) {
        float avg = (currents[0] + currents[1] + currents[2] + currents[3]) / 4.0f;
        bool all_ok = true;
        for (int i = 0; i < 4; i++) {
            if (currents[i] < avg * 0.6f) { all_ok = false; break; }
        }
        if (all_ok && hal_ms() - blade_loss.timer_ms > 2000U) {
            blade_loss.active = false;
            blade_loss.damaged_idx = -1;
            for (int i = 0; i < 4; i++) blade_loss.efficiency[i] = 1.0f;
        }
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
    ADC1->RSQ1 = (2 << 20);
    ADC1->RSQ2 = (11 << 0) | (12 << 5) | (13 << 10);
    ADC1->CTRL2 = (1<<8)|(1<<1)|(1<<22);

    if (bmi270_config_upload() != 0) {
        /* Config upload failed — sensor may run without optimized config */
    }
    bmi270_real_init();
    bmi270_calibrate();
    dshot_init_all();
    crsf_init();
    flash_log_init();

    pid_init(pid.kp, pid.ki, pid.kd, pid.kf, pid.out, pid.i, pid.gp, pid.dl);
    pg_init(&pg.ip, &pg.didt, &pg.dilp, &pg.thr, &pg.mismatch,
            &pg.until, &pg.cool, &pg.axis, &pg.scale, &pg.ev);
    for (int a = 0; a < 3; a++) {
        base_kp[a] = pid.kp[a]; base_ki[a] = pid.ki[a]; base_kd[a] = pid.kd[a];
    }
    filt_init(filt.n, 3200.0f, FRAME_HZ);
    rpm_init(&rpm.model, &rpm.meas, &rpm.fused, &rpm.conf, rpm.hz);
    iterm_init(iterm.i, 400.0f);

    /* Advanced safety init */
    pred_ff_init(pid.kf);
    crash_rec_init();
    blade_loss_init();
    cur_notch_init();
    rls_autotune_init();

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