#pragma once
#include <stdint.h>
#include <stdbool.h>

typedef struct {
    uint16_t erpm;
    uint8_t  temperature;
    uint16_t voltage;
    uint16_t current;
    uint8_t  crc_error;
    uint8_t  decode_error;
    bool     valid;
    uint32_t lastUpdateUs;
} esc_telemetry_t;

void dshot_bidir_init(void);
void dshot_bidir_write(uint8_t motor, uint16_t value, bool requestTelemetry);
void dshot_bidir_request_telemetry_all(void);
bool dshot_bidir_process_telemetry(void);
const esc_telemetry_t* dshot_bidir_get_telemetry(uint8_t motor);
bool dshot_bidir_telemetry_alive(void);
