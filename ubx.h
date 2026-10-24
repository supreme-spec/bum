#pragma once
#include <stdint.h>
#include <stdbool.h>

typedef enum {
    UBX_FIX_NONE = 0,
    UBX_FIX_DEAD_RECKONING = 1,
    UBX_FIX_2D = 2,
    UBX_FIX_3D = 3,
    UBX_FIX_GNSS_DR = 4,
    UBX_FIX_TIME_ONLY = 5,
} ubx_fix_type_e;

typedef struct {
    int32_t lat_e7;
    int32_t lon_e7;
    int32_t alt_msl_mm;
    uint32_t h_acc_mm;
    uint32_t v_acc_mm;
    int32_t vel_n_mms;
    int32_t vel_e_mms;
    int32_t vel_d_mms;
    int32_t gspeed_mms;
    int32_t head_mot_e5;
    ubx_fix_type_e fix_type;
    uint8_t num_sv;
    bool valid_fix;
    uint32_t last_pvt_us;
    uint32_t last_sat_us;
} ubx_snapshot_t;

void ubx_init(void);
void ubx_process(void);
const ubx_snapshot_t* ubx_get_snapshot(void);
void ubx_configure_messages(void);
void failsafe_update_gps_from_ubx(void);
