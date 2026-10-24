#include "radun.h"

#define BMI_GYR_X_L  0x0C
#define BMI_ACC_X_L  0x12
#define BMI_TEMP_L   0x22
#define BMI_GYR_CONF 0x42
#define BMI_GYR_RANGE 0x43
#define BMI_INT1_IO  0x53
#define BMI_INT_MAP  0x58
#define BMI_PWR_CTRL 0x7D
#define BMI_CMD      0x7E

static int16_t gyro_offset[3] = {0, 0, 0};
static bool calibrated = false;
static bool healthy = false;

static int abs_i(int v) { return v < 0 ? -v : v; }

void bmi270_real_init(void) {
    gpio_af(GYRO_CS_PORT, GPIO_PIN_5, 5); gpio_af(GYRO_CS_PORT, GPIO_PIN_6, 5); gpio_af(GYRO_CS_PORT, GPIO_PIN_7, 5);
    gpio_out(GYRO_CS_PORT, GYRO_CS_PIN); gpio_set(GYRO_CS_PORT, GYRO_CS_PIN);
    spi_init(SPI1);

    gpio_clr(GYRO_CS_PORT, GYRO_CS_PIN);
    spi_rw(SPI1, BMI_CMD | 0x80); spi_rw(SPI1, 0xB6);
    gpio_set(GYRO_CS_PORT, GYRO_CS_PIN); delay_ms(2);

    gpio_clr(GYRO_CS_PORT, GYRO_CS_PIN);
    spi_rw(SPI1, BMI_GYR_CONF | 0x80); spi_rw(SPI1, 0x2D);
    spi_rw(SPI1, BMI_GYR_RANGE | 0x80); spi_rw(SPI1, 0x00);
    spi_rw(SPI1, BMI_INT1_IO | 0x80);   spi_rw(SPI1, 0x0A);
    spi_rw(SPI1, BMI_INT_MAP | 0x80);   spi_rw(SPI1, 0x04);
    spi_rw(SPI1, BMI_PWR_CTRL | 0x80);  spi_rw(SPI1, 0x06);
    gpio_set(GYRO_CS_PORT, GYRO_CS_PIN); delay_ms(50);

    uint8_t id = 0;
    gpio_clr(GYRO_CS_PORT, GYRO_CS_PIN);
    spi_rw(SPI1, 0x00 | 0x80);
    id = spi_rw(SPI1, 0xFF);
    gpio_set(GYRO_CS_PORT, GYRO_CS_PIN);
    healthy = (id == 0x24);
}

void bmi270_process(imu_t *imu) {
    gpio_clr(GYRO_CS_PORT, GYRO_CS_PIN);
    spi_rw(SPI1, BMI_GYR_X_L | 0x80);
    int16_t g[3], a[3];
    for (int i = 0; i < 6; i++) {
        uint8_t l = spi_rw(SPI1, 0xFF), h = spi_rw(SPI1, 0xFF);
        int16_t v = (int16_t)((uint16_t)l | ((uint16_t)h << 8));
        if (i < 3) g[i] = v; else a[i-3] = v;
    }
    gpio_set(GYRO_CS_PORT, GYRO_CS_PIN);

    if (calibrated) {
        g[0] -= gyro_offset[0]; g[1] -= gyro_offset[1]; g[2] -= gyro_offset[2];
    }

    imu->dps[0] = -(float)g[1] * (2000.0f/32768.0f);
    imu->dps[1] =  (float)g[0] * (2000.0f/32768.0f);
    imu->dps[2] =  (float)g[2] * (2000.0f/32768.0f);
    imu->acc[0] = (float)a[0] * (8.0f/32768.0f);
    imu->acc[1] = (float)a[1] * (8.0f/32768.0f);
    imu->acc[2] = (float)a[2] * (8.0f/32768.0f);

    gpio_clr(GYRO_CS_PORT, GYRO_CS_PIN);
    spi_rw(SPI1, BMI_TEMP_L | 0x80);
    uint8_t tl = spi_rw(SPI1, 0xFF), th = spi_rw(SPI1, 0xFF);
    gpio_set(GYRO_CS_PORT, GYRO_CS_PIN);
    imu->temp = 23.0f + (float)(int16_t)((uint16_t)tl | ((uint16_t)th << 8)) / 512.0f;
    imu->healthy = healthy;
}

void bmi270_calibrate(void) {
    int32_t sum[3] = {0, 0, 0};
    uint32_t cnt = 0;
    delay_ms(500);
    for (uint32_t i = 0; i < 6400; i++) {
        imu_t tmp;
        bmi270_process(&tmp);
        sum[0] += (int16_t)(tmp.dps[0] * 32768.0f / 2000.0f);
        sum[1] += (int16_t)(tmp.dps[1] * 32768.0f / 2000.0f);
        sum[2] += (int16_t)(tmp.dps[2] * 32768.0f / 2000.0f);
        cnt++;
        delay_us(312);
    }
    int16_t avg[3] = {(int16_t)(sum[0]/cnt), (int16_t)(sum[1]/cnt), (int16_t)(sum[2]/cnt)};
    if (abs_i(avg[0]) < 48 && abs_i(avg[1]) < 48 && abs_i(avg[2]) < 48) {
        gyro_offset[0] = avg[0]; gyro_offset[1] = avg[1]; gyro_offset[2] = avg[2];
        calibrated = true;
    }
}

bool bmi270_is_healthy(void) { return healthy; }