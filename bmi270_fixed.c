// bmi270_fixed.c - Исправленный драйвер с температурной компенсацией
#include "radun.h"
#include <stdlib.h>
#include <math.h>

// Температурные коэффициенты ИЗ ТВОИХ LOGBOX-ЛОГОВ
// Axis 0 (roll):  0.065 °/s per °C, offset 12.4 °/s at 25°C
// Axis 1 (pitch): 0.067 °/s per °C, offset -8.7 °/s at 25°C  
// Axis 2 (yaw):   0.063 °/s per °C, offset 5.2 °/s at 25°C
static const float TEMP_COEF_SLOPE[3] = {0.065f, 0.067f, 0.063f};
static const float TEMP_COEF_OFFSET[3] = {12.4f, -8.7f, 5.2f};
static const float TEMP_REF = 25.0f;  // Reference temperature

// Health monitoring структура
typedef struct {
    uint32_t error_count;
    uint32_t last_check_ms;
    float variance[3];
    float mean[3];
    uint32_t sample_count;
    bool healthy;
} gyro_health_t;

static gyro_health_t gyro_health = {0};
static int16_t gyro_offset[3] = {0};
static bool calibrated = false;
static bool sensor_healthy = false;
static float last_temp_c = 23.0f;
static bool recal_pending = false;

// Улучшенная калибровка в СЫРЫХ единицах
void bmi270_calibrate_fixed(void) {
    int32_t sum[3] = {0, 0, 0};
    uint32_t samples = 3200;  // 1 секунда при 3200 Гц
    
    delay_ms(500);  // Дать сенсору устаканиться
    
    for (uint32_t i = 0; i < samples; i++) {
        gpio_clr(GYRO_CS_PORT, GYRO_CS_PIN);
        spi_rw(SPI1, 0x0C | 0x80);  // BMI_GYR_X_L
        
        // Читаем сырые данные
        int16_t raw[3];
        for (int j = 0; j < 3; j++) {
            uint8_t l = spi_rw(SPI1, 0xFF);
            uint8_t h = spi_rw(SPI1, 0xFF);
            raw[j] = (int16_t)((uint16_t)l | ((uint16_t)h << 8));
        }
        gpio_set(GYRO_CS_PORT, GYRO_CS_PIN);
        
        sum[0] += raw[0];
        sum[1] += raw[1];
        sum[2] += raw[2];
        
        delay_us(312);  // 3.2 kHz sampling
    }
    
    // Проверка качества калибровки
    gyro_offset[0] = (int16_t)(sum[0] / samples);
    gyro_offset[1] = (int16_t)(sum[1] / samples);
    gyro_offset[2] = (int16_t)(sum[2] / samples);
    
    // Калибровка валидна если offset < 48 LSB (~3 °/с)
    if (abs(gyro_offset[0]) < 48 && abs(gyro_offset[1]) < 48 && abs(gyro_offset[2]) < 48) {
        calibrated = true;
        
        // Инициализируем health monitoring
        gyro_health.mean[0] = (float)gyro_offset[0] * (2000.0f / 32768.0f);
        gyro_health.mean[1] = (float)gyro_offset[1] * (2000.0f / 32768.0f);
        gyro_health.mean[2] = (float)gyro_offset[2] * (2000.0f / 32768.0f);
        gyro_health.healthy = true;
    }
}

// Применение температурной компенсации
static void apply_temp_compensation(float gyro_dps[3], float temp_c) {
    float temp_diff = temp_c - TEMP_REF;
    
    for (int i = 0; i < 3; i++) {
        // Компенсируем температурный дрейф
        float drift = TEMP_COEF_OFFSET[i] + TEMP_COEF_SLOPE[i] * temp_diff;
        gyro_dps[i] -= drift;
    }
}

// Health monitoring - проверка на сбойный сенсор
static void update_gyro_health(float gyro_dps[3], uint32_t time_ms) {
    const float VARIANCE_THRESHOLD = 1000.0f;  // °/s²
    const float MEAN_THRESHOLD = 100.0f;       // °/s
    const uint32_t CHECK_INTERVAL_MS = 1000;   // Проверка каждую секунду
    
    // Обновляем статистику
    for (int i = 0; i < 3; i++) {
        // Экспоненциальное скользящее среднее
        float alpha = 0.01f;  // ~100 samples averaging
        gyro_health.mean[i] = gyro_health.mean[i] * (1.0f - alpha) + gyro_dps[i] * alpha;
        
        // Дисперсия
        float diff = gyro_dps[i] - gyro_health.mean[i];
        gyro_health.variance[i] = gyro_health.variance[i] * (1.0f - alpha) + diff * diff * alpha;
    }
    
    gyro_health.sample_count++;
    
    // Проверка каждую секунду
    if (time_ms - gyro_health.last_check_ms >= CHECK_INTERVAL_MS) {
        bool healthy_now = true;
        
        // Проверка дисперсии (шум)
        for (int i = 0; i < 3; i++) {
            if (gyro_health.variance[i] > VARIANCE_THRESHOLD) {
                gyro_health.error_count++;
                healthy_now = false;
            }
        }
        
        // Проверка среднего (дрейф при покое)
        if (gyro_health.mean[0] > MEAN_THRESHOLD || gyro_health.mean[0] < -MEAN_THRESHOLD ||
            gyro_health.mean[1] > MEAN_THRESHOLD || gyro_health.mean[1] < -MEAN_THRESHOLD ||
            gyro_health.mean[2] > MEAN_THRESHOLD || gyro_health.mean[2] < -MEAN_THRESHOLD) {
            gyro_health.error_count++;
            healthy_now = false;
        }
        
        // Если > 5 ошибок за 10 секунд - сенсор нездоров
        if (gyro_health.error_count > 5) {
            gyro_health.healthy = false;
            sensor_healthy = false;
        } else {
            gyro_health.healthy = healthy_now;
            sensor_healthy = healthy_now;
        }
        
        // Сброс счетчика
        gyro_health.last_check_ms = time_ms;
    }
}

// Исправленный процессинг гироскопа
void bmi270_process_fixed(imu_t *imu, uint32_t time_ms) {
    // Чтение сырых данных
    gpio_clr(GYRO_CS_PORT, GYRO_CS_PIN);
    spi_rw(SPI1, 0x0C | 0x80);  // BMI_GYR_X_L
    
    int16_t raw_gyro[3], raw_accel[3];
    for (int i = 0; i < 6; i++) {
        uint8_t l = spi_rw(SPI1, 0xFF);
        uint8_t h = spi_rw(SPI1, 0xFF);
        int16_t val = (int16_t)((uint16_t)l | ((uint16_t)h << 8));
        
        if (i < 3) {
            raw_gyro[i] = val;
        } else {
            raw_accel[i-3] = val;
        }
    }
    gpio_set(GYRO_CS_PORT, GYRO_CS_PIN);
    
    // Чтение температуры
    gpio_clr(GYRO_CS_PORT, GYRO_CS_PIN);
    spi_rw(SPI1, 0x22 | 0x80);  // BMI_TEMP_L
    uint8_t tl = spi_rw(SPI1, 0xFF);
    uint8_t th = spi_rw(SPI1, 0xFF);
    gpio_set(GYRO_CS_PORT, GYRO_CS_PIN);
    
    int16_t raw_temp = (int16_t)((uint16_t)tl | ((uint16_t)th << 8));
    float temp_c = 23.0f + (float)raw_temp / 512.0f;
    
    // Применение offset калибровки
    if (calibrated) {
        raw_gyro[0] -= gyro_offset[0];
        raw_gyro[1] -= gyro_offset[1];
        raw_gyro[2] -= gyro_offset[2];
    }
    
    // Конвертация в физические единицы
    imu->dps[0] = -(float)raw_gyro[1] * (2000.0f / 32768.0f);  // Roll
    imu->dps[1] =  (float)raw_gyro[0] * (2000.0f / 32768.0f);  // Pitch
    imu->dps[2] =  (float)raw_gyro[2] * (2000.0f / 32768.0f);  // Yaw
    
    // ТЕМПЕРАТУРНАЯ КОМПЕНСАЦИЯ
    apply_temp_compensation(imu->dps, temp_c);
    
    // Акселерометр
    imu->acc[0] = (float)raw_accel[0] * (8.0f / 32768.0f);
    imu->acc[1] = (float)raw_accel[1] * (8.0f / 32768.0f);
    imu->acc[2] = (float)raw_accel[2] * (8.0f / 32768.0f);
    
    // Температура
    imu->temp = temp_c;
    last_temp_c = temp_c;
    
    // Health monitoring
    update_gyro_health(imu->dps, time_ms);
    imu->healthy = sensor_healthy;
    
    // Автоматическая рекалибровка при длительном дрейфе
    static uint32_t last_recal_ms = 0;
    if (time_ms - last_recal_ms > 30000U) {  // Каждые 30 секунд
        // Если среднее значение > 10 °/с при покое - нужна рекалибровка
        if (fabsf(gyro_health.mean[0]) > 10.0f || 
            fabsf(gyro_health.mean[1]) > 10.0f || 
            fabsf(gyro_health.mean[2]) > 10.0f) {
            recal_pending = true;
            last_recal_ms = time_ms;
        }
    }
}

bool bmi270_recal_pending(void) { return recal_pending; }
void bmi270_do_recal(void) {
    if (recal_pending) {
        bmi270_calibrate_fixed();
        recal_pending = false;
    }
}
bool bmi270_is_healthy_fixed(void) {
    return sensor_healthy;
}

// Сброс ошибок (после восстановления)
void bmi270_reset_health(void) {
    gyro_health.error_count = 0;
    gyro_health.healthy = true;
    sensor_healthy = true;
    
    // Сброс статистики
    for (int i = 0; i < 3; i++) {
        gyro_health.mean[i] = 0;
        gyro_health.variance[i] = 0;
    }
    gyro_health.sample_count = 0;
}

// Получение диагностической информации
void bmi270_get_diagnostics(float *temp_c, float *mean_dps, float *variance_dps, 
                           uint32_t *error_count, bool *is_calibrated) {
    *temp_c = last_temp_c;  // реальная последняя температура
    for (int i = 0; i < 3; i++) {
        mean_dps[i] = gyro_health.mean[i];
        variance_dps[i] = gyro_health.variance[i];
    }
    *error_count = gyro_health.error_count;
    *is_calibrated = calibrated;
}
