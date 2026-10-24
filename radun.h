#pragma once
#include "radun_fix/radun_hw.h"

/* ============================================================================
 * radun.h — проектный заголовок. Тянет корректную карту регистров AT32F435
 * из radun_fix/radun_hw.h и добавляет высокоуровневые типы/декларации.
 * ============================================================================ */

/* --- тактирование (целевые значения для PLL-режима; фактические — в hal) --- */
#define SYSCLK_HZ       288000000UL
#define APB1_HZ         144000000UL
#define APB2_HZ         144000000UL
#define VREF_MV         3280U

#define VBAT_DIVIDER    10
#define VBAT_SCALE      102
#ifndef VBAT_FACT
#define VBAT_FACT       8.45f
#endif
#define FRAME_HZ        112.0f

/* --- пины: обратная совместимость со старым кодом (GPIO_PIN_* == PIN_*) --- */
#define GPIO_PIN_0  PIN_0
#define GPIO_PIN_1  PIN_1
#define GPIO_PIN_2  PIN_2
#define GPIO_PIN_3  PIN_3
#define GPIO_PIN_4  PIN_4
#define GPIO_PIN_5  PIN_5
#define GPIO_PIN_6  PIN_6
#define GPIO_PIN_7  PIN_7
#define GPIO_PIN_8  PIN_8
#define GPIO_PIN_9  PIN_9
#define GPIO_PIN_10 PIN_10
#define GPIO_PIN_11 PIN_11
#define GPIO_PIN_12 PIN_12
#define GPIO_PIN_13 PIN_13
#define GPIO_PIN_14 PIN_14
#define GPIO_PIN_15 PIN_15

/* --- периферия платы --- */
#define FLASH_SPI       SPI2
#define FLASH_CS_PORT   GPIOB
#define FLASH_CS_PIN    GPIO_PIN_12

#define GYRO_CS_PORT    GPIOA
#define GYRO_CS_PIN     GPIO_PIN_4

/* ============================================================================
 * Высокоуровневые типы
 * ============================================================================ */
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

typedef struct { float v; float i; } bat_t;

typedef struct { float alt; } baro_t;

typedef struct {
    float kp[3], ki[3], kd[3], kf[3];
    float out[3], i[3], gp[3], dl[3];
}
pid_t;

typedef struct { float sp[3]; } sp_t;
typedef struct { float model, meas, fused, conf, hz[3]; } rpm_t;
typedef struct { float i[3]; } iterm_t;
typedef struct { float ip, didt, dilp, thr, mismatch, scale; uint32_t until, cool; int8_t axis; uint16_t ev; } pg_t;
typedef struct { float n[3][3][5]; } filt_t;

typedef struct { float last_setpoint[3]; float jerk[3]; float kf_pred[3]; } pred_ff_t;
typedef struct { bool detected; uint32_t timer_ms; uint32_t start_ms; } crash_rec_t;
typedef struct { float efficiency[4]; bool active; int8_t damaged_idx; uint32_t timer_ms; } blade_loss_t;
typedef struct { float estimated_hz; float filtered_current; } current_notch_t;

extern pred_ff_t pred_ff;
extern crash_rec_t crash_rec;
extern blade_loss_t blade_loss;
extern current_notch_t cur_notch;

/* ============================================================================
 * Декларации функций (реализация в соответствующих .c)
 * ============================================================================ */
void bmi270_real_init(void);
void bmi270_process(imu_t *imu);
void bmi270_calibrate(void);
int bmi270_config_upload(void);
bool bmi270_is_healthy(void);
void bmi270_set_gyro_rate(uint16_t odr);
uint16_t bmi270_get_gyro_rate(void);
void dshot_telem_feed(int motor, uint32_t pulse_us);
float dshot_get_rpm(int motor);
void crsf_init(void);
void crsf_poll(crsf_t* out);
void pid_init(float kp[3], float ki[3], float kd[3], float kf[3], float out[3], float i[3], float gp[3], float dl[3]);
void pid_update(float kp[3], float ki[3], float kd[3], float kf[3], float sp[3], float dps[3], float dt, float thr, float out[3], float i[3], float gp[3], float dl[3]);
void pid_set_d_weight(float weight[3]);
void modes_apply(crsf_t* rx, float att[2], float sp[3]);
void mixer(float out[3], uint16_t thr, uint16_t m[4]);
void failsafe(crsf_t* rx, bool* active);
bool arming_interlock_update(const crsf_t *rx, bool failsafe_active);
void prearm_init(void);
bool prearm_check(const imu_t *imu, const crsf_t *rx, const bat_t *bat);
void prearm_get_status(bool *all_ok, uint8_t *failed_mask);
float vbat_sag_compensate(float vbat, float current);
float anti_gravity_scale(uint16_t thr);
void turtle_mode_mix(uint16_t m[4], bool active);
void launch_control(float sp[3], uint16_t thr, bool active);
void imu_update_attitude(float dps[3], float acc[3], float att[2], float dt);
void filt_init(float n[3][3][5], float fs, float frame);
void filt_apply(float n[3][3][5], float dps[3]);
void filt_notch_update(float n[3][3][5], int axis, int stage, float fc, float q, float fs);
void acc_filter_init(void);
void acc_filter_update(const float acc[3], uint32_t time_ms);
float acc_filter_get_freq(int axis);
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

/* Battery management */
void battery_init(void);
void battery_update(float vbat, float current);
float battery_get_compensated_voltage(void);
bool battery_low_voltage(void);
uint8_t battery_get_cell_count(void);

/* USB CDC/VCP */
void usb_cdc_init(void);
bool usb_cdc_is_connected(void);
void usb_cdc_send(const uint8_t *data, uint16_t len);
uint16_t usb_cdc_receive(uint8_t *data, uint16_t max_len);
void usb_cdc_poll(void);

/* CLI interface */
void cli_init(void);
void cli_feed(uint8_t c);
void cli_print(const char *str);

void flash_log_init(void);
void flash_log(uint32_t loop_cnt, const imu_t *imu, const pid_t *pid, const uint16_t motors[4], const bat_t *bat);
void flash_log_start(void);
void flash_log_stop(void);
uint32_t flash_get_dropped(void);

/* HAL */
void hal_init(void);
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

/* Sensor diagnostics and dynamic filters */
void bmi270_process_fixed(imu_t *imu, uint32_t time_ms);
bool bmi270_is_healthy_fixed(void);
bool bmi270_calibrate_fixed(void);
void bmi270_get_diagnostics(float *temp_c, float *mean_dps, float *variance_dps,
                            uint32_t *error_count, bool *is_calibrated);
void filter_bank_init(float frame_hz, float fs, float slew_rate);
void filter_bank_update_rpm(float rpm_hz, float rpm_confidence, uint32_t time_ms);
void filter_bank_apply(float dps[3]);
void filter_bank_reset(void);
void fft_update_gyro(float gyro_val);
void fft_process(void);
int fft_get_peaks(float *out, int maxn);
void propguard_init(float threshold, float mismatch, uint32_t cooldown_ms);
void propguard_update(float current, float gyro[3], float sp[3], uint32_t time_ms);
float propguard_get_scale(int axis);
void propguard_reset(void);
void propguard_get_diagnostics(int8_t *triggered_axis, float *scale,
                               uint32_t *event_count, float *current_dilp);
