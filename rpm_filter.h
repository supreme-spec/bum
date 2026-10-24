#pragma once
#include <stdint.h>
#include <stdbool.h>

#define RPM_FILTER_MOTORS 4
#define RPM_FILTER_AXES 3
#define RPM_FILTER_HARMONICS 3
#define MOTOR_POLES 14
#define RPM_FILTER_MIN_HZ 80.0f
#define RPM_FILTER_MAX_HZ 2500.0f
#define RPM_FILTER_Q 2.8f
#define RPM_FILTER_LPF_HZ 150.0f
#define RPM_FILTER_UPDATE_HZ 2000

void rpm_filter_init(void);
void rpm_filter_update_telemetry(uint8_t motor, uint16_t erpm);
void rpm_filter_feed_erpm(uint8_t motor, uint16_t erpm);
void rpm_filter_update_coeffs(void);
float rpm_filter_apply(uint8_t axis, float in);
bool rpm_filter_active(uint8_t motor);
bool rpm_filter_healthy(void);
