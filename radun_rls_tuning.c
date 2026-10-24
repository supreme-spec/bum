#include "radun.h"
#include <math.h>

static float rls_P[3] = {1e6f, 1e6f, 1e6f};
static float rls_theta[3] = {1.0f, 1.0f, 1.0f};
static float rls_lambda = 0.99f;

void rls_autotune_init(void) {
    rls_P[0] = rls_P[1] = rls_P[2] = 1e6f;
    rls_theta[0] = rls_theta[1] = rls_theta[2] = 1.0f;
    rls_lambda = 0.99f;
}

float rls_autotune_update(int axis, float current_omega, float pid_output) {
    if (axis < 0 || axis > 2) return 1.0f;
    
    float x = pid_output;
    float y = current_omega;
    
    float K = rls_P[axis] * x / (rls_lambda + x * rls_P[axis] * x);
    
    float error = y - rls_theta[axis] * x;
    
    rls_theta[axis] = rls_theta[axis] + K * error;
    
    rls_P[axis] = (1.0f / rls_lambda) * (rls_P[axis] - K * x * rls_P[axis]);
    
    float scale = 1.0f / rls_theta[axis];
    
    if (scale > 2.0f) scale = 2.0f;
    if (scale < 0.5f) scale = 0.5f;
    
    return scale;
}

void flash_log_init(void) {
    RCC->AHB1 |= (1 << 1);
    gpio_out(GPIOB, GPIO_PIN_12);
    spi_init(SPI2);
    gpio_clr(GPIOB, GPIO_PIN_12);
    delay_us(100);
    gpio_set(GPIOB, GPIO_PIN_12);
}

void flash_log(uint32_t loop_cnt, const imu_t *imu, const pid_t *pid, const uint16_t motors[4], const bat_t *bat) {
    static uint8_t buf[32];
    static uint32_t addr = 0;
    
    if (addr > 0x7FFF) return;
    
    buf[0] = 0x02;
    buf[1] = 0x03;
    buf[2] = (loop_cnt >> 8) & 0xFF;
    buf[3] = loop_cnt & 0xFF;
    
    for (int i = 0; i < 3; i++) {
        int16_t dps_val = (int16_t)(imu->dps[i] * 16.0f);
        buf[4 + i*2] = (dps_val >> 8) & 0xFF;
        buf[5 + i*2] = dps_val & 0xFF;
    }
    
    for (int i = 0; i < 3; i++) {
        int16_t pid_val = (int16_t)(pid->out[i] * 10.0f);
        buf[10 + i*2] = (pid_val >> 8) & 0xFF;
        buf[11 + i*2] = pid_val & 0xFF;
    }
    
    for (int i = 0; i < 4; i++) {
        uint16_t m = motors[i];
        buf[16 + i*2] = (m >> 8) & 0xFF;
        buf[17 + i*2] = m & 0xFF;
    }
    
    uint16_t vbat_val = (uint16_t)(bat->v * 100.0f);
    buf[24] = (vbat_val >> 8) & 0xFF;
    buf[25] = vbat_val & 0xFF;
    
    buf[26] = imu->healthy ? 1 : 0;
    buf[27] = 0xAA;
    
    gpio_clr(FLASH_CS_PORT, FLASH_CS_PIN);
    spi_rw(FLASH_SPI, 0x06);
    gpio_set(FLASH_CS_PORT, FLASH_CS_PIN);
    
    delay_us(10);
    gpio_clr(FLASH_CS_PORT, FLASH_CS_PIN);
    spi_rw(FLASH_SPI, 0x02);
    spi_rw(FLASH_SPI, (addr >> 16) & 0xFF);
    spi_rw(FLASH_SPI, (addr >> 8) & 0xFF);
    spi_rw(FLASH_SPI, addr & 0xFF);
    
    for (int i = 0; i < 32; i++) {
        spi_rw(FLASH_SPI, buf[i]);
    }
    
    gpio_set(FLASH_CS_PORT, FLASH_CS_PIN);
    addr += 32;
}

void flash_log_start(void) {
    gpio_clr(FLASH_CS_PORT, FLASH_CS_PIN);
    spi_rw(FLASH_SPI, 0x06);
    gpio_set(FLASH_CS_PORT, FLASH_CS_PIN);
    
    delay_us(10);
    gpio_clr(FLASH_CS_PORT, FLASH_CS_PIN);
    spi_rw(FLASH_SPI, 0xC7);
    gpio_set(FLASH_CS_PORT, FLASH_CS_PIN);
}

void flash_log_stop(void) {
    gpio_clr(FLASH_CS_PORT, FLASH_CS_PIN);
    spi_rw(FLASH_SPI, 0x04);
    gpio_set(FLASH_CS_PORT, FLASH_CS_PIN);
}