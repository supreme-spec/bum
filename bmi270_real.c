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
static uint16_t gyro_odr = 3200;  /* Default ODR */

static int abs_i(int v) { return v < 0 ? -v : v; }

void bmi270_real_init(void) {
    gpio_af(GYRO_CS_PORT, GPIO_PIN_5, 5); gpio_af(GYRO_CS_PORT, GPIO_PIN_6, 5); gpio_af(GYRO_CS_PORT, GPIO_PIN_7, 5);
    gpio_out(GYRO_CS_PORT, GYRO_CS_PIN); gpio_set(GYRO_CS_PORT, GYRO_CS_PIN);
    spi_init(SPI1);

    /* Configure the sensors only after bmi270_config_upload() succeeds. */
    gpio_clr(GYRO_CS_PORT, GYRO_CS_PIN);
    spi_rw(SPI1, BMI_GYR_CONF); spi_rw(SPI1, 0x2D);
    spi_rw(SPI1, BMI_GYR_RANGE); spi_rw(SPI1, 0x00);
    spi_rw(SPI1, BMI_INT1_IO);   spi_rw(SPI1, 0x0A);
    spi_rw(SPI1, BMI_INT_MAP);   spi_rw(SPI1, 0x04);
    spi_rw(SPI1, BMI_PWR_CTRL);  spi_rw(SPI1, 0x06);
    gpio_set(GYRO_CS_PORT, GYRO_CS_PIN); delay_ms(50);

    uint8_t id = 0;
    gpio_clr(GYRO_CS_PORT, GYRO_CS_PIN);
    spi_rw(SPI1, 0x00 | 0x80);
    (void)spi_rw(SPI1, 0xFF);
    id = spi_rw(SPI1, 0xFF);
    gpio_set(GYRO_CS_PORT, GYRO_CS_PIN);
    healthy = (id == 0x24);
}

void bmi270_process(imu_t *imu) {
    /* Gyro sync optimization: burst read with minimal CS toggling */
    gpio_clr(GYRO_CS_PORT, GYRO_CS_PIN);
    delay_us(1);  /* CS setup time */

    /* Read gyro + accel in one burst (12 bytes) */
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
    (void)spi_rw(SPI1, 0xFF);
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

bool bmi270_is_healthy(void) {
    return healthy;
}

void bmi270_set_gyro_rate(uint16_t odr) {
    /* BMI270 supported ODRs: 25, 50, 100, 200, 400, 800, 1600, 3200 Hz */
    if (odr == 25 || odr == 50 || odr == 100 || odr == 200 ||
        odr == 400 || odr == 800 || odr == 1600 || odr == 3200) {
        gyro_odr = odr;
        /* Note: Requires reconfiguration of BMI270 registers */
        /* For now, just store the value */
    }
}

uint16_t bmi270_get_gyro_rate(void) {
    return gyro_odr;
}