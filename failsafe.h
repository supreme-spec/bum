#pragma once
#include <stdint.h>
#include <stdbool.h>

typedef enum {
    FS_NORMAL = 0,
    FS_WARNING,
    FS_CRITICAL
} fs_state_e;

typedef struct {
    bool rc_lost;
    bool gps_lost;
    bool gyro_stuck;
    bool vbat_low;
    bool vbat_crit;
} fs_flags_t;

extern fs_flags_t fs_flags;
extern fs_state_e fs_state;

void failsafe_init(void);
void failsafe_update(void);
bool failsafe_allow_arm(void);
void failsafeUpdateGps(double lat, double lon, float alt, uint8_t sats, float speedMs);
void failsafe_update_gps_from_ubx(void);
