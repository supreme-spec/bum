#include "failsafe.h"
#include "radun.h"

fs_flags_t fs_flags;
fs_state_e fs_state = FS_NORMAL;

static uint32_t last_check_fs = 0;

void failsafe_init(void) {
    fs_flags.rc_lost = false;
    fs_flags.gyro_stuck = false;
    fs_flags.vbat_low = false;
    fs_flags.vbat_crit = false;
    fs_state = FS_NORMAL;
    last_check_fs = hal_ms();
}

void failsafe_update(void) {
    if (hal_ms() - last_check_fs < 100) return;
    last_check_fs = hal_ms();

    extern bat_t bat;
    extern crsf_t rx;
    extern imu_t imu;
    (void)rx;
    (void)imu;

    if (bat.v < 14.4f) fs_flags.vbat_low = true;
    else fs_flags.vbat_low = false;
    if (bat.v < 13.6f) fs_flags.vbat_crit = true;
    else fs_flags.vbat_crit = false;

    if (fs_flags.rc_lost || fs_flags.vbat_crit || fs_flags.gyro_stuck) {
        fs_state = FS_CRITICAL;
    } else if (fs_flags.vbat_low) {
        fs_state = FS_WARNING;
    } else {
        fs_state = FS_NORMAL;
    }
}

bool failsafe_allow_arm(void) {
    extern crsf_t rx;
    extern imu_t imu;
    if (!rx.valid) return false;
    if (!imu.healthy) return false;
    if (fs_flags.vbat_crit) return false;
    return true;
}
