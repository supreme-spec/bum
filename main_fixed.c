/* main_fixed.c — полная интеграция всех подсистем (исправленная, P0+) */
#include "radun.h"
#include "board_accif435.h"
#include <math.h>
#include "watchdog.h"
#include "failsafe.h"
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

/* ---------- Arm FSM ---------- */
typedef enum { ST_DISARMED, ST_ARM_REQUEST, ST_ARMED, ST_FAULT } arm_state_t;
static arm_state_t arm_st = ST_DISARMED;
static float arm_t = 0.0f;

static void arming_task(float dt, const crsf_t *rx, bool prearm_ok, bool fs)
{
    bool stick_low   = rx->thr < 1050;
    bool gesture     = rx->y > 700 && rx->r < -200 && rx->p < -200;
    bool kill_gesture= rx->y < -700 && stick_low;

    switch (arm_st) {
    case ST_DISARMED:
        arm_t = 0.0f;
        if (stick_low && gesture && prearm_ok && !fs) arm_st = ST_ARM_REQUEST;
        break;
    case ST_ARM_REQUEST:      /* жест надо удерживать 300 мс */
        arm_t += dt;
        if (!(stick_low && gesture) || fs || !prearm_ok) arm_st = ST_DISARMED;
        else if (arm_t > 0.3f) { arm_st = ST_ARMED; pid.i[0] = pid.i[1] = pid.i[2] = 0.0f; filter_bank_reset(); }
        break;
    case ST_ARMED:
        if (fs) arm_st = ST_FAULT;
        else if (kill_gesture) { arm_t += dt; if (arm_t > 0.5f) arm_st = ST_DISARMED; }
        else arm_t = 0.0f;
        break;
    case ST_FAULT:
        if (!fs && stick_low) arm_st = ST_DISARMED;
        break;
    }
}

void TIM6_DAC_IRQHandler(void) {
    uint32_t current_ms = hal_ms();
    float dt = 1.0f / 3200.0f;
    TMR6->INTF = 0;

    bmi270_process_fixed(&imu, current_ms);
    filter_bank_apply(imu.dps);
    imu_update_attitude(imu.dps, imu.acc, imu.att, dt);
    crsf_poll(&rx);

    failsafe(&rx, &fs_active);
    bool prearm_ok = prearm_check(&imu, &rx, &bat);
    arming_task(dt, &rx, prearm_ok, fs_active);
    armed = (arm_st == ST_ARMED);

    if (fs_active && armed) {
        armed = false;
        arm_st = ST_FAULT;
        pid.i[0] = pid.i[1] = pid.i[2] = 0;
        pid.out[0] = pid.out[1] = pid.out[2] = 0;
        flash_log_stop();
    }

    float sp_rate[3] = {0, 0, 0};
    if (armed) {
        bool is_crashed = crash_rec_check(imu.dps, rx.thr, armed);
        if (is_crashed) {
            crash_rec_reset_pid(pid.i);
        } else {
            modes_apply(&rx, imu.att, sp_rate);
        }
    }

    if (armed) {
        /* РОВНО ОДИН вызов PID на такт */
        pid_update(pid.kp, pid.ki, pid.kd, pid.kf, sp_rate, imu.dps,
                   dt, (float)rx.thr, pid.out, pid.i, pid.gp, pid.dl);
        for (int axis = 0; axis < 3; axis++) {
            if (pid.out[axis] >  1000.0f) pid.out[axis] =  1000.0f;
            if (pid.out[axis] < -1000.0f) pid.out[axis] = -1000.0f;
        }
    } else {
        sp.sp[0] = sp.sp[1] = sp.sp[2] = 0;
        pid.out[0] = pid.out[1] = pid.out[2] = 0;
    }

    asymmetric_mixer(pid.out, (uint16_t)rx.thr, motors, blade_loss.efficiency);

    if (armed && !fs_active && bmi_config_loaded && imu.healthy &&
        dshot_output_available()) {
        dshot_send(motors);
    } else {
        uint16_t off[4] = {0, 0, 0, 0};
        dshot_send(off);
    }

    if (++bb_cnt >= 8) {
        bb_cnt = 0;
        flash_log(loop_cnt, &imu, &pid, motors, &bat);
    }

    loop_cnt++;
}

void TIM7_IRQHandler(void) {
    uint32_t current_ms = hal_ms();
    TMR7->INTF = 0;

    /* ADC читается циклически через DMA в adc_dma_update() (главный цикл) */
    bat.v = ((float)adc_get_vbat_raw() / 4095.0f) * 3.28f * VBAT_FACT;

    if (current_ms - last_health_check_ms >= 1000U) {
        last_health_check_ms = current_ms;
        if (!bmi270_is_healthy_fixed()) {
            armed = false;
            arm_st = ST_DISARMED;
            fs_active = true;
            pid.i[0] = pid.i[1] = pid.i[2] = 0;
            pid.out[0] = pid.out[1] = pid.out[2] = 0;
            uint16_t off[4] = {0, 0, 0, 0};
            dshot_send(off);
        }
    }

    IWDG->KR = IWDG_KEY_RELOAD;

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

    /* SPI1 для гироскопа (SCK=PB3, MISO=PB4, MOSI=PB5, CS=PA4) */
    spi_init(SPI1);
    gpio_af(SPI1_SCK_GPIO, SPI1_SCK_PIN, SPI1_AF);
    gpio_af(SPI1_MISO_GPIO, SPI1_MISO_PIN, SPI1_AF);
    gpio_af(SPI1_MOSI_GPIO, SPI1_MOSI_PIN, SPI1_AF);
    gpio_out(GYRO_CS_PORT, GYRO_CS_PIN);
    gpio_set(GYRO_CS_PORT, GYRO_CS_PIN);

    /* I2C1 для барометра (SCL=PB6, SDA=PB7) */
    gpio_af(I2C1_SCL_GPIO, I2C1_SCL_PIN, I2C1_AF);
    gpio_af(I2C1_SDA_GPIO, I2C1_SDA_PIN, I2C1_AF);
    I2C1->CTRL2 = (uint32_t)(g_pclk1_hz / 1000000UL);
    I2C1->CTRL1 = (1UL << 15) | (1UL << 0);

    /* ADC1: канал 11 (VBAT), циклически через DMA */
    ADC1->SMPT1 = 0x0FFFFFFF;
    ADC1->RSQ1  = (1UL << 20) | (ADC_CTRL2_ADCON);   /* 1 канал, SQ1 = канал 11 */
    ADC1->RSQ2  = 0;

    bmi_config_loaded = bmi270_config_upload() == 0;
    if (bmi_config_loaded) {
        bmi270_real_init();
        bmi_config_loaded = bmi270_calibrate_fixed();
    }
    dshot_init();
    crsf_init();
    flash_log_init();
    adc_dma_init();

    pid_init(pid.kp, pid.ki, pid.kd, pid.kf, pid.out, pid.i, pid.gp, pid.dl);
    filter_bank_init(FRAME_HZ, 3200.0f, 200.0f);
    crash_rec_init();
    blade_loss_init();

    /* IWDG: перезагрузка ~0.8 с, кормим из TIM7 */
    IWDG->KR = IWDG_KEY_ENABLE;
    IWDG->KR = IWDG_KEY_WRITE;
    IWDG->PR = 4;
    IWDG->RLR = 1000;
    IWDG->KR = IWDG_KEY_RELOAD;

    watchdog_init(200);
    failsafe_init();
    rpm_filter_init();
    ubx_init();

    /* TIM6 — контур управления 3.2 кГц (IRQ 53), TIM7 — ADC/LED 800 Гц (IRQ 54) */
    NVIC_SetPriority(53, 0);
    NVIC_SetPriority(54, 1);
    CORE_NVIC_ISER1 = (1UL << (53 - 32)) | (1UL << (54 - 32));

    TMR6->PSC  = (tmrclk1_hz() / 3200U) - 1U;
    TMR6->CARR = 0;
    TMR6->CTRL1 = (1UL << 2) | (1UL << 0);   /* URS=1, CEN=1 */
    TMR6->DINTEN = 1;

    TMR7->PSC  = (tmrclk1_hz() / 800U) - 1U;
    TMR7->CARR = 0;
    TMR7->CTRL1 = (1UL << 2) | (1UL << 0);
    TMR7->DINTEN = 1;

    for (int i = 0; i < 3; i++) {
        gpio_set(LED_PORT, LED_PIN);
        delay_ms(100);
        gpio_clr(LED_PORT, LED_PIN);
        delay_ms(100);
    }

    while (1) {
        adc_dma_update();
        flash_log_process();
        watchdog_feed();
        __asm volatile ("wfi");
    }
}
