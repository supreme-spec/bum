#include "radun.h"
#include <math.h>

typedef struct {
    uint32_t error_count;
    uint32_t sample_count;
    float mean[3];
    float variance[3];
} gyro_diagnostics_t;

static gyro_diagnostics_t gyro_diagnostics;
static int16_t gyro_offset[3];
static bool calibrated;
static bool sensor_healthy;
static float last_temp_c = 23.0f;

bool bmi270_calibrate_fixed(void) {
    calibrated = false;
    sensor_healthy = false;
    gyro_diagnostics.sample_count = 0;
    for (int axis = 0; axis < 3; axis++) {
        gyro_diagnostics.mean[axis] = 0.0f;
        gyro_diagnostics.variance[axis] = 0.0f;
    }
    if (!bmi270_is_healthy()) return false;

    int32_t sum[3] = {0, 0, 0};
    uint64_t sum_sq[3] = {0, 0, 0};
    const uint32_t samples = 3200;

    delay_ms(500);
    for (uint32_t sample = 0; sample < samples; sample++) {
        int16_t raw[3];
        gpio_clr(GYRO_CS_PORT, GYRO_CS_PIN);
        spi_rw(SPI1, 0x0C | 0x80);
        (void)spi_rw(SPI1, 0xFF);
        for (int axis = 0; axis < 3; axis++) {
            uint8_t low = spi_rw(SPI1, 0xFF);
            uint8_t high = spi_rw(SPI1, 0xFF);
            raw[axis] = (int16_t)((uint16_t)low | ((uint16_t)high << 8));
        }
        gpio_set(GYRO_CS_PORT, GYRO_CS_PIN);

        for (int axis = 0; axis < 3; axis++) {
            sum[axis] += raw[axis];
            sum_sq[axis] += (uint64_t)((int32_t)raw[axis] * raw[axis]);
        }
        delay_us(312);
    }

    for (int axis = 0; axis < 3; axis++) {
        int32_t average = sum[axis] / (int32_t)samples;
        uint64_t mean_square = sum_sq[axis] / samples;
        uint64_t average_square = (uint64_t)(average * average);
        if (average < -48 || average > 48 ||
            mean_square < average_square ||
            mean_square - average_square > 2500U) {
            gyro_diagnostics.error_count++;
            return false;
        }
        gyro_offset[axis] = (int16_t)average;
    }

    calibrated = true;
    sensor_healthy = true;
    return true;
}

static void update_diagnostics(const float gyro_dps[3]) {
    const float alpha = 0.01f;
    for (int axis = 0; axis < 3; axis++) {
        float delta = gyro_dps[axis] - gyro_diagnostics.mean[axis];
        gyro_diagnostics.mean[axis] += alpha * delta;
        gyro_diagnostics.variance[axis] +=
            alpha * (delta * delta - gyro_diagnostics.variance[axis]);
    }
    gyro_diagnostics.sample_count++;
}

void bmi270_process_fixed(imu_t *imu, uint32_t time_ms) {
    (void)time_ms;
    int16_t raw_gyro[3];
    int16_t raw_accel[3];

    gpio_clr(GYRO_CS_PORT, GYRO_CS_PIN);
    spi_rw(SPI1, 0x0C | 0x80);
    (void)spi_rw(SPI1, 0xFF);
    for (int index = 0; index < 6; index++) {
        uint8_t low = spi_rw(SPI1, 0xFF);
        uint8_t high = spi_rw(SPI1, 0xFF);
        int16_t value = (int16_t)((uint16_t)low | ((uint16_t)high << 8));
        if (index < 3) raw_gyro[index] = value;
        else raw_accel[index - 3] = value;
    }
    gpio_set(GYRO_CS_PORT, GYRO_CS_PIN);

    gpio_clr(GYRO_CS_PORT, GYRO_CS_PIN);
    spi_rw(SPI1, 0x22 | 0x80);
    (void)spi_rw(SPI1, 0xFF);
    uint8_t temp_low = spi_rw(SPI1, 0xFF);
    uint8_t temp_high = spi_rw(SPI1, 0xFF);
    gpio_set(GYRO_CS_PORT, GYRO_CS_PIN);

    int16_t raw_temp = (int16_t)((uint16_t)temp_low |
                                 ((uint16_t)temp_high << 8));
    last_temp_c = 23.0f + (float)raw_temp / 512.0f;

    if (calibrated) {
        for (int axis = 0; axis < 3; axis++) {
            raw_gyro[axis] -= gyro_offset[axis];
        }
    }

    const float gyro_scale = 2000.0f / 32768.0f;
    const float accel_scale = 8.0f / 32768.0f;

    /* ACCIF435 Betaflight alignment is CW90 plus an additional 900 mdeg yaw. */
    imu->dps[0] = -(float)raw_gyro[0] * gyro_scale;
    imu->dps[1] = -(float)raw_gyro[1] * gyro_scale;
    imu->dps[2] =  (float)raw_gyro[2] * gyro_scale;
    imu->acc[0] = -(float)raw_accel[0] * accel_scale;
    imu->acc[1] = -(float)raw_accel[1] * accel_scale;
    imu->acc[2] =  (float)raw_accel[2] * accel_scale;
    imu->temp = last_temp_c;

    update_diagnostics(imu->dps);
    imu->healthy = sensor_healthy;
}

bool bmi270_is_healthy_fixed(void) {
    return sensor_healthy;
}

void bmi270_get_diagnostics(float *temp_c, float *mean_dps,
                            float *variance_dps, uint32_t *error_count,
                            bool *is_calibrated) {
    *temp_c = last_temp_c;
    for (int axis = 0; axis < 3; axis++) {
        mean_dps[axis] = gyro_diagnostics.mean[axis];
        variance_dps[axis] = gyro_diagnostics.variance[axis];
    }
    *error_count = gyro_diagnostics.error_count;
    *is_calibrated = calibrated;
}
