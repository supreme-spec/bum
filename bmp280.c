#include "radun.h"

static int32_t t_fine;

static uint32_t bmp280_read24(uint8_t reg) {
    i2c_write(0x76, reg, 0);
    uint32_t msb = i2c_read(0x76, reg);
    uint32_t lsb = i2c_read(0x76, reg+1);
    uint32_t xlsb = i2c_read(0x76, reg+2);
    return (msb << 16) | (lsb << 8) | xlsb;
}

static uint16_t bmp280_read16(uint8_t reg) {
    i2c_write(0x76, reg, 0);
    uint16_t msb = i2c_read(0x76, reg);
    uint16_t lsb = i2c_read(0x76, reg+1);
    return (msb << 8) | lsb;
}

void bmp280_init(void) {
    i2c_write(0x76, 0xF4, 0x6F);
    i2c_write(0x76, 0xF5, 0x80);
}

void bmp280_read(baro_t *baro) {
    int32_t adc_T = (int32_t)bmp280_read24(0xFA);
    
    static int32_t dig_T1 = 27504, dig_T2 = 26435, dig_T3 = -1000;
    
    int32_t var1 = ((((adc_T >> 3) - (dig_T1 << 1))) * dig_T2) >> 11;
    int32_t var2 = (((((adc_T >> 4) - dig_T1) * ((adc_T >> 4) - dig_T1)) >> 12) * dig_T3) >> 14;
    t_fine = var1 + var2;
    
    int32_t T = (t_fine * 5 + 128) >> 8;
    
    int32_t adc_P = (int32_t)bmp280_read24(0xF7);
    static int64_t dig_P1 = 36477, dig_P2 = -10685, dig_P3 = 3024, dig_P4 = 2855,
                   dig_P5 = 140, dig_P6 = -7, dig_P7 = 15500, dig_P8 = -14600,
                   dig_P9 = 6000;
    
    var1 = ((int64_t)t_fine) - 128000;
    var2 = var1 * var1 * dig_P6;
    var2 += var1 * dig_P5 << 17;
    var2 += dig_P4 << 35;
    var1 = var1 * var1 * dig_P3 >> 8;
    var1 += dig_P2 << 12;
    var1 = var1 * var1 >> 1;
    
    if (var1 != 0) {
        int64_t p = 1048576 - adc_P;
        p = (((p << 31) - var2) * 3125) / var1;
        var1 = (dig_P9 * (p >> 13) * (p >> 13)) >> 25;
        var2 = (dig_P8 * p) >> 19;
        p = (p + var1 + var2) >> 8;
        p += dig_P7 << 4;
        baro->alt = (float)(p / 256.0) * 0.01f;
    } else {
        baro->alt = 0.0f;
    }
}