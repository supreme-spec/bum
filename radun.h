#pragma once
#include <stdint.h>
#include <stdbool.h>

#define SYSCLK_HZ       288000000UL
#define APB1_HZ         72000000UL
#define APB2_HZ         144000000UL
#define VREF_MV         3280U

#define PERIPH_BASE     0x40000000UL
#define AHB1PERIPH      (PERIPH_BASE + 0x00020000)
#define APB1PERIPH      (PERIPH_BASE + 0x00000000)
#define APB2PERIPH      (PERIPH_BASE + 0x00010000)

typedef struct { volatile uint32_t CTRL, PLL, CTRL2, APB1, APB2, AHB1, AHB2, AHB3,
    _r0, APB1RP, APB2RP, _r1[2], BDCTRL, CTRLSTS, AHB1RP, AHB2RP, AHB3RP,
    _r2[2], APB1RP2, APB2RP2; }
RCC_t;
#define RCC ((RCC_t*)(AHB1PERIPH + 0x3800))

typedef struct { volatile uint32_t MODE, OTYPE, OSPEED, PUPD, IN, OUT, BSET, BCLR,
    LOCK, AFSEL[2]; }
GPIO_t;
#define GPIOA ((GPIO_t*)(AHB1PERIPH + 0x0000))
#define GPIOB ((GPIO_t*)(AHB1PERIPH + 0x0400))
#define GPIOC ((GPIO_t*)(AHB1PERIPH + 0x0800))

typedef struct { volatile uint32_t CTRL1, CTRL2, STS, DT, _r[4]; }
SPI_t;
#define SPI1 ((SPI_t*)(APB2PERIPH + 0x3000))
#define SPI2 ((SPI_t*)(APB1PERIPH + 0x3800))

typedef struct { volatile uint32_t STS, DT, BAUD, CTRL1, CTRL2, CTRL3, GTPR; }
UART_t;
#define USART6 ((UART_t*)(APB2PERIPH + 0x1400))

typedef struct { volatile uint32_t CTRL1, CTRL2, SMCTRL, DINTEN, INTF, SWEV,
    CHCTRL1, CHCTRL2, CHCCTRL, CNT, PSC, CARR, RPT, CH1C, CH2C, CH3C, CH4C,
    BDCTRL, DMACFG, DMATB; }
TIM_t;
#define TIM1 ((TIM_t*)(APB2PERIPH + 0x0000))
#define TIM8 ((TIM_t*)(APB2PERIPH + 0x13400))
#define TIM3 ((TIM_t*)(APB1PERIPH + 0x0400))
#define TIM6 ((TIM_t*)(APB1PERIPH + 0x1000))
#define TIM7 ((TIM_t*)(APB1PERIPH + 0x1400))

typedef struct { volatile uint32_t STS, CTRL1, CTRL2, SMPT1, SMPT2, _r[6],
    RSQ1, RSQ2, RSQ3, _r2[4], RDAT; }
ADC_t;
#define ADC1 ((ADC_t*)(APB2PERIPH + 0x2000))

#define NVIC_ISER0 (*(volatile uint32_t*)0xE000E100UL)
#define NVIC_ISER1 (*(volatile uint32_t*)0xE000E104UL)
#define NVIC_IPR(n) (*(volatile uint8_t*)(0xE000E400UL + (n)))

typedef struct { volatile uint32_t INTEN, EVTEN, RTEN, FTEN, SWIEV, INTF; }
EXTI_t;
#define EXTI ((EXTI_t*)(APB2PERIPH + 0x3C00))

typedef struct { volatile uint32_t CTRL1, CTRL2, OAR1, _r, DR, STS1, STS2; }
I2C_t;
#define I2C1 ((I2C_t*)(APB1PERIPH + 0x5400))

typedef struct { volatile uint32_t KR, PR, RLR, SR; }
IWDG_t;
#define IWDG ((IWDG_t*)0x40003000UL)
#define FLASH_ACR (*(volatile uint32_t*)0x40023C00UL)

#define GPIO_PIN_4   (1<<4)
#define GPIO_PIN_5   (1<<5)
#define GPIO_PIN_6   (1<<6)
#define GPIO_PIN_7   (1<<7)
#define GPIO_PIN_8   (1<<8)
#define GPIO_PIN_9   (1<<9)
#define GPIO_PIN_12  (1<<12)
#define GPIO_PIN_13  (1<<13)
#define GPIO_PIN_14  (1<<14)
#define GPIO_PIN_15  (1<<15)

#define FLASH_SPI       SPI2
#define FLASH_CS_PORT   GPIOB
#define FLASH_CS_PIN    GPIO_PIN_12

#define GYRO_CS_PORT    GPIOA
#define GYRO_CS_PIN     GPIO_PIN_4

#define VBAT_DIVIDER    10
#define VBAT_SCALE      102
/* Калибровка делителя батареи под ACCIF435 (Vbat = (vraw/4095)*VREF*VBAT_FACT) */
#ifndef VBAT_FACT
#define VBAT_FACT       8.45f
#endif
#define FRAME_HZ        112.0f

typedef struct { int16_t r, p, y, thr; uint16_t aux; uint8_t lq; bool valid; }
crsf_t;

typedef struct {
    float dps[3];
    float acc[3];
    float att[2];
    float temp;
    bool healthy;
}
imu_t;

typedef struct {
    float v;
    float i;
}
bat_t;

typedef struct {
    float alt;
}
baro_t;

typedef struct {
    float kp[3], ki[3], kd[3], kf[3];
    float out[3], i[3], gp[3], dl[3];
}
pid_t;

typedef struct { float sp[3]; }
sp_t;
typedef struct { float model, meas, fused, conf, hz[3]; }
rpm_t;
typedef struct { float i[3]; }
iterm_t;
typedef struct { float ip, didt, dilp, thr, mismatch, scale; uint32_t until, cool; int8_t axis; uint16_t ev; }
pg_t;
typedef struct { float n[3][3][5]; }
filt_t;

/* Advanced safety types */
typedef struct { float last_setpoint[3]; float jerk[3]; float kf_pred[3]; }
pred_ff_t;
typedef struct { bool detected; uint32_t timer_ms; uint32_t start_ms; }
crash_rec_t;
typedef struct { float efficiency[4]; bool active; int8_t damaged_idx; uint32_t timer_ms; }
blade_loss_t;
typedef struct { float estimated_hz; float filtered_current; }
current_notch_t;

extern pred_ff_t pred_ff;
extern crash_rec_t crash_rec;
extern blade_loss_t blade_loss;
extern current_notch_t cur_notch;

void hal_init(void);
uint32_t hal_ms(void);
void delay_ms(uint32_t);
void delay_us(uint32_t);
void gpio_out(GPIO_t*, uint16_t);
void gpio_af(GPIO_t*, uint16_t, uint8_t);
void gpio_set(GPIO_t*, uint16_t);
void gpio_clr(GPIO_t*, uint16_t);
uint8_t spi_rw(SPI_t*, uint8_t);
void spi_init(SPI_t* s);
void i2c_write(uint8_t, uint8_t, uint8_t);
uint8_t i2c_read(uint8_t, uint8_t);
void uart_init(UART_t*, uint32_t);
void NVIC_SetPriority(uint8_t IRQn, uint32_t priority);

void bmi270_real_init(void);
void bmi270_process(imu_t *imu);
void bmi270_calibrate(void);
int bmi270_config_upload(void);
bool bmi270_is_healthy(void);
void dshot_init_all(void);
void dshot_send(uint16_t motors[4]);
bool dshot_output_available(void);
void dshot_telem_feed(int motor, uint32_t pulse_us);
float dshot_get_rpm(int motor);
void crsf_init(void);
void crsf_poll(crsf_t* out);
void pid_init(float kp[3], float ki[3], float kd[3], float kf[3], float out[3], float i[3], float gp[3], float dl[3]);
void pid_update(float kp[3], float ki[3], float kd[3], float kf[3], float sp[3], float dps[3], float dt, float thr, float out[3], float i[3], float gp[3], float dl[3]);
void modes_apply(crsf_t* rx, float att[2], float sp[3]);
void mixer(float out[3], uint16_t thr, uint16_t m[4]);
void failsafe(crsf_t* rx, bool* active);
bool arming_interlock_update(const crsf_t *rx, bool failsafe_active);
float vbat_sag_compensate(float vbat, float current);
float anti_gravity_scale(uint16_t thr);
void turtle_mode_mix(uint16_t m[4], bool active);
void launch_control(float sp[3], uint16_t thr, bool active);
void imu_update_attitude(float dps[3], float acc[3], float att[2], float dt);
void filt_init(float n[3][3][5], float fs, float frame);
void filt_apply(float n[3][3][5], float dps[3]);
void filt_notch_update(float n[3][3][5], int axis, int stage, float fc, float q, float fs);
void rpm_init(float* model, float* meas, float* fused, float* conf, float hz[3]);
void rpm_update(float* model, float* meas, float* fused, float* conf, float hz[3], float vbat, float duty);
void iterm_init(float i[3], float lim);
float iterm_upd(float i[3], int a, float ki, float err, float sp, float dt, bool sat);
void pg_init(float* ip, float* didt, float* dilp, float* thr, float* mismatch,
             uint32_t* until, uint32_t* cool, int8_t* axis, float* scale, uint16_t* ev);
void pg_upd(float ip, float didt, float dilp, float thr, float mismatch,
            uint32_t until, uint32_t cool, int8_t axis, float scale, uint16_t ev,
            float i, float gyro[3], float sp[3], uint32_t ms);
float pg_scale(float scale, int8_t axis, int a);
void bmp280_init(void);
void bmp280_read(baro_t *baro);

void flash_log_init(void);
void flash_log(uint32_t loop_cnt, const imu_t *imu, const pid_t *pid, const uint16_t motors[4], const bat_t *bat);
void flash_log_start(void);
void flash_log_stop(void);
void msp_process(uint8_t cmd, uint8_t *payload, uint8_t len);

/* Advanced safety */
void pred_ff_init(float kf[3]);
void pred_ff_update(float sp[3], float ff_out[3], float dt);
void crash_rec_init(void);
bool crash_rec_check(float gyro[3], uint16_t throttle, bool armed);
void crash_rec_reset_pid(float i[3]);
bool nn_crash_detect(float gyro[3]);
void blade_loss_init(void);
void blade_loss_detect(const float currents[4], const float gyro[3]);
void asymmetric_mixer(float out[3], uint16_t thr, uint16_t m[4], const float eff[4]);
void cur_notch_init(void);
float cur_notch_update(float duty, float vbat, int pole_pairs);

/* RLS autotune - returns scale factor (caller applies vs base gains) */
float rls_autotune_update(int axis, float current_omega, float pid_output);
void rls_autotune_init(void);

/* Register blocks for Cortex-M core */
typedef struct { volatile uint32_t CTRL, CYCCNT; }
DWT_t;
#define DWT ((DWT_t*)0xE0001000UL)
#define SCB ((volatile uint32_t*)0xE000ED00UL)
typedef struct { volatile uint32_t DEMCR; }
CoreDebug_t;
#define CoreDebug ((CoreDebug_t*)0xE000EDF0UL)
typedef struct { volatile uint32_t CTRL, LOAD, VAL, CALIB; }
SysTick_t;
#define SysTick ((SysTick_t*)0xE000E010UL)
/* Sensor diagnostics and dynamic filters */

void bmi270_process_fixed(imu_t *imu, uint32_t time_ms);
bool bmi270_is_healthy_fixed(void);
bool bmi270_calibrate_fixed(void);
void bmi270_get_diagnostics(float *temp_c, float *mean_dps, float *variance_dps, 
                           uint32_t *error_count, bool *is_calibrated);

/* Банк фильтров с управлением состоянием */
void filter_bank_init(float frame_hz, float fs, float slew_rate);
void filter_bank_update_rpm(float rpm_hz, float rpm_confidence, uint32_t time_ms);
void filter_bank_apply(float dps[3]);
void filter_bank_reset(void);
void fft_update_gyro(float gyro_val);
void fft_process(void);
int fft_get_peaks(float *out, int maxn);

/* Рабочий PropGuard */
void propguard_init(float threshold, float mismatch, uint32_t cooldown_ms);
void propguard_update(float current, float gyro[3], float sp[3], uint32_t time_ms);
float propguard_get_scale(int axis);
void propguard_reset(void);
void propguard_get_diagnostics(int8_t *triggered_axis, float *scale, 
                              uint32_t *event_count, float *current_dilp);
