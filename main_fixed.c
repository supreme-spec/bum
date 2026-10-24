// main_fixed.c - Исправленная версия с температурной компенсацией и PropGuard
#include "radun.h"
#include "radun_rls_tuning.h"
#include <math.h>

// Исправленные компоненты
void bmi270_process_fixed(imu_t *imu, uint32_t time_ms);
bool bmi270_is_healthy_fixed(void);
void bmi270_calibrate_fixed(void);
void bmi270_reset_health(void);

void filter_bank_init(float frame_hz, float fs, float slew_rate);
void filter_bank_update_rpm(float rpm_hz, float rpm_confidence, uint32_t time_ms);
void filter_bank_apply(float dps[3]);
void filter_bank_reset(void);
void filter_bank_get_frequencies(float fc[3][3]);

void propguard_init(float threshold, float mismatch, uint32_t cooldown_ms);
void propguard_update(float current, float gyro[3], float sp[3], uint32_t time_ms);
float propguard_get_scale(int axis);
void propguard_reset(void);

// Глобальное состояние
imu_t imu;
crsf_t rx;
bat_t bat;
baro_t baro;
pid_t pid;
sp_t sp;
rpm_t rpm;
iterm_t iterm;
uint16_t motors[4];
bool armed = false, fs_active = false;

// Время и счетчики
static uint32_t loop_cnt = 0, bb_cnt = 0;
static uint32_t last_rpm_update_ms = 0;
static uint32_t last_health_check_ms = 0;
static uint32_t boot_time_ms = 0;

// Диагностика
float filter_freqs[3][3];
float current = 0.0f;  // Placeholder - нужно добавить измерение тока

void TIM6_DAC_IRQHandler(void) {
    uint32_t current_ms = hal_ms();
    TIM6->INTF = 0;

    /* 1. Чтение гироскопа с температурной компенсацией */
    bmi270_process_fixed(&imu, current_ms);

    /* 2. Применение фильтров */
    filter_bank_apply(imu.dps);

    /* 3. Расчет углов (complementary filter) */
    imu_update_attitude(imu.dps, imu.acc, imu.att, 1.0f/3200.0f);

    /* 4. Приемник */
    crsf_poll(&rx);

    /* 5. Arming/Disarming с проверкой здоровья IMU */
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

    /* 6. Crash Recovery (исправленные пороги) */
    bool is_crashed = false;
    if (armed) {
        // Более реалистичные пороги: 2000 °/с при низком газе
        bool extreme_spin = (fabsf(imu.dps[0]) > 2000.0f ||
                            fabsf(imu.dps[1]) > 2000.0f ||
                            fabsf(imu.dps[2]) > 2000.0f);
        bool low_throttle = (rx.thr < 1050);
        
        is_crashed = extreme_spin && low_throttle;
        
        if (is_crashed) {
            pid.i[0] = pid.i[1] = pid.i[2] = 0;
            sp.sp[0] = sp.sp[1] = sp.sp[2] = 0;
        } else {
            modes_apply(&rx, imu.att, sp.sp);
        }
    }

    /* 7. PropGuard - защита от удара пропеллером */
    propguard_update(current, imu.dps, sp.sp, current_ms);

    /* 8. PID update */
    if (armed && !is_crashed) {
        float dt = 1.0f / 3200.0f;
        
        for (int axis = 0; axis < 3; axis++) {
            float error = sp.sp[axis] - imu.dps[axis];
            
            // Smart I-term с релаксацией
            static float lp[3] = {0};
            float alpha = dt / (dt + 1.0f / (6.2831853f * 15.0f));
            lp[axis] += alpha * (sp.sp[axis] - lp[axis]);
            float hp = sp.sp[axis] - lp[axis];
            float relax = 1.0f / (1.0f + (hp*hp) / 810000.0f);
            
            pid.i[axis] += pid.ki[axis] * error * 0.01f * relax * dt;
            
            // Anti-windup
            if (error * pid.i[axis] > 0.0f) {
                pid.i[axis] *= 0.995f;
            }
            if (pid.i[axis] > 400.0f) pid.i[axis] = 400.0f;
            if (pid.i[axis] < -400.0f) pid.i[axis] = -400.0f;
            
            // P-term
            float p_term = pid.kp[axis] * error * 0.01f;
            
            // D-term с TPA
            float tpa = 1.0f;
            if (rx.thr > 1500) {
                tpa = 1.0f - 0.5f * ((float)(rx.thr - 1500) / 500.0f);
            }
            
            float d_raw = -(imu.dps[axis] - pid.gp[axis]) / dt;
            pid.dl[axis] += 0.15f * (d_raw - pid.dl[axis]);
            float d_term = pid.kd[axis] * pid.dl[axis] * 0.0001f * tpa;
            
            // Feed-forward
            float ff = pid.kf[axis] * sp.sp[axis] * 0.01f;
            
            // Суммирование
            pid.out[axis] = p_term + pid.i[axis] + d_term + ff;
            pid.gp[axis] = imu.dps[axis];
            
            // Применение PropGuard scaling
            float pg_scale = propguard_get_scale(axis);
            pid.out[axis] *= pg_scale;
            
            // Ограничение
            if (pid.out[axis] > 1000.0f) pid.out[axis] = 1000.0f;
            if (pid.out[axis] < -1000.0f) pid.out[axis] = -1000.0f;
        }
    } else {
        sp.sp[0] = sp.sp[1] = sp.sp[2] = 0;
        pid.out[0] = pid.out[1] = pid.out[2] = 0;
    }

    /* 9. Failsafe */
    failsafe(&rx, &fs_active);

    /* 10. Mixer */
    mixer(pid.out, (uint16_t)rx.thr, motors);

    /* 11. DShot output */
    if (armed && !fs_active && imu.healthy) {
        dshot_send(motors);
    } else {
        uint16_t off[4] = {48, 48, 48, 48};
        dshot_send(off);
    }

    /* 12. Blackbox logging @800Hz */
    if (++bb_cnt >= 4) {
        bb_cnt = 0;
        flash_log(loop_cnt, &imu, &pid, motors, &bat);
        
        // Также логируем состояние фильтров и PropGuard
        filter_bank_get_frequencies(filter_freqs);
        // flash_log_diagnostics(filter_freqs, propguard_get_scale(0), current);
    }

    loop_cnt++;
}

void TIM7_IRQHandler(void) {
    uint32_t current_ms = hal_ms();
    TIM7->INTF = 0;

    /* 1. Чтение напряжения батареи */
    ADC1->CTRL2 |= (1 << 22);
    while (ADC1->STS & (1 << 1));
    uint16_t vraw = ADC1->RDAT;
    bat.v = ((float)vraw / 4095.0f) * 3.28f * 8.5f;

    /* 2. Обновление RPM модели для фильтров */
    float duty = (float)(rx.thr - 1000) / 1000.0f;
    rpm_update(&rpm.model, &rpm.meas, &rpm.fused, &rpm.conf, rpm.hz, bat.v, duty);
    
    // Обновляем фильтры RPM каждые 100ms
    if (current_ms - last_rpm_update_ms >= 100U) {
        last_rpm_update_ms = current_ms;
        filter_bank_update_rpm(rpm.fused, rpm.conf, current_ms);
    }

    /* 3. Health monitoring - проверка IMU каждую секунду */
    if (current_ms - last_health_check_ms >= 1000U) {
        last_health_check_ms = current_ms;
        
        if (!bmi270_is_healthy_fixed()) {
            // IMU нездоров - пытаемся восстановить
            bmi270_reset_health();
            if (!imu.healthy) {
                // Критическая ошибка - disarm
                armed = false;
                uint16_t off[4] = {48, 48, 48, 48};
                dshot_send(off);
            }
        }
    }

    /* 3b. Отложенная рекалибровка гироскопа (вне ISR) */
    if (!armed && bmi270_recal_pending()) {
        bmi270_do_recal();
    }
    IWDG->KR = 0xAAAA;

    /* 5. LED индикация статуса */
    static uint32_t led_timer = 0;
    if (current_ms - led_timer >= 1000U) {
        led_timer = current_ms;
        
        if (!imu.healthy) {
            // Быстрое мигание при ошибке IMU
            gpio_set(GPIOC, GPIO_PIN_14);
            delay_ms(50);
            gpio_clr(GPIOC, GPIO_PIN_14);
            delay_ms(50);
            gpio_set(GPIOC, GPIO_PIN_14);
            delay_ms(50);
            gpio_clr(GPIOC, GPIO_PIN_14);
        } else if (armed) {
            // Медленное мигание при armed
            gpio_set(GPIOC, GPIO_PIN_14);
            delay_ms(100);
            gpio_clr(GPIOC, GPIO_PIN_14);
        } else {
            // Одиночное мигание при standby
            gpio_set(GPIOC, GPIO_PIN_14);
            delay_ms(10);
            gpio_clr(GPIOC, GPIO_PIN_14);
        }
    }
}

int main(void) {
    hal_init();
    boot_time_ms = hal_ms();

    // Инициализация периферии
    spi_init(SPI1);
    spi_init(SPI2);
    
    gpio_af(GPIOB, GPIO_PIN_8|GPIO_PIN_9, 4);
    I2C1->CTRL2 = APB1_HZ / 1000000;
    I2C1->CTRL1 = (1<<15)|(1<<0);
    
    ADC1->SMPT1 = 0x0FFFFFFF;
    ADC1->RSQ1 = (2 << 20);
    ADC1->RSQ2 = (11 << 0) | (12 << 5) | (13 << 10);
    ADC1->CTRL2 = (1<<8)|(1<<1)|(1<<22);

    // Инициализация датчиков и драйверов
    if (bmi270_config_upload() != 0) {
        // fallback: продолжаем с дефолтной инициализацией
    }
    bmi270_real_init();
    bmi270_calibrate_fixed();
    dshot_init_all();
    crsf_init();
    flash_log_init();

    // Инициализация алгоритмов
    pid_init(pid.kp, pid.ki, pid.kd, pid.kf, pid.out, pid.i, pid.gp, pid.dl);
    filter_bank_init(FRAME_HZ, 3200.0f, 8.0f);  // 8 Hz/sec slew rate
    propguard_init(9000.0f, 300.0f, 100);       // Пороги из твоих логов
    rpm_init(&rpm.model, &rpm.meas, &rpm.fused, &rpm.conf, rpm.hz);
    iterm_init(iterm.i, 400.0f);

    // Watchdog
    IWDG->KR = 0x5555; IWDG->PR = 4; IWDG->RLR = 1000;
    IWDG->KR = 0xAAAA; IWDG->KR = 0xCCCC;

    // Настройка прерываний
    NVIC_SetPriority(54, 0);  // TIM6 - высокий приоритет
    NVIC_SetPriority(55, 1);  // TIM7 - средний приоритет
    NVIC_ISER1 = (1 << 22) | (1 << 23);  // Enable TIM6 и TIM7

    // Настройка таймеров
    TIM6->PSC = (APB1_HZ / 3200) - 1;  // 3.2 kHz
    TIM6->CARR = 0;
    TIM6->CTRL1 = (1<<3)|(1<<0);  // One pulse mode
    TIM6->DINTEN = 1;             // Enable interrupt
    
    TIM7->PSC = (APB1_HZ / 800) - 1;   // 800 Hz  
    TIM7->CARR = 0;
    TIM7->CTRL1 = (1<<3)|(1<<0);
    TIM7->DINTEN = 1;

    // Индикация запуска
    for (int i = 0; i < 3; i++) {
        gpio_set(GPIOC, GPIO_PIN_14);
        delay_ms(100);
        gpio_clr(GPIOC, GPIO_PIN_14);
        delay_ms(100);
    }

    // Главный цикл (прерывания делают всю работу)
    while (1) {
        asm volatile("wfi");  // Ожидание прерываний
    }
}


