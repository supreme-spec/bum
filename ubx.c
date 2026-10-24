#include "ubx.h"
#include "radun.h"
#include "failsafe.h"
#include <string.h>

#define UBX_SYNC1 0xB5
#define UBX_SYNC2 0x62
#define UBX_CLASS_NAV 0x01
#define UBX_ID_PVT 0x07
#define UBX_ID_SAT 0x35
#define UBX_CLASS_CFG 0x06
#define UBX_ID_CFG_MSG 0x01

typedef enum {
    UBX_WAIT_SYNC1 = 0,
    UBX_WAIT_SYNC2,
    UBX_WAIT_CLASS,
    UBX_WAIT_ID,
    UBX_WAIT_LEN_LO,
    UBX_WAIT_LEN_HI,
    UBX_WAIT_PAYLOAD,
    UBX_WAIT_CK_A,
    UBX_WAIT_CK_B,
} ubx_state_e;

static ubx_state_e state = UBX_WAIT_SYNC1;
static uint8_t msg_class, msg_id;
static uint16_t payload_len, payload_idx;
static uint8_t payload[256];
static uint8_t ck_a, ck_b;
static uint8_t exp_ck_a, exp_ck_b;
static ubx_snapshot_t snapshot;

static void fletcher_reset(void) { ck_a = 0; ck_b = 0; }
static void fletcher_add(uint8_t b) { ck_a += b; ck_b += ck_a; }

static int32_t rd_i32(const uint8_t *p) {
    return (int32_t)((uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24));
}
static uint32_t rd_u32(const uint8_t *p) {
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static void parse_nav_pvt(const uint8_t *p) {
    if (!p) return;
    snapshot.fix_type = (ubx_fix_type_e)p[20];
    snapshot.lon_e7 = rd_i32(&p[24]);
    snapshot.lat_e7 = rd_i32(&p[28]);
    snapshot.alt_msl_mm = rd_i32(&p[36]);
    snapshot.h_acc_mm = rd_u32(&p[40]);
    snapshot.v_acc_mm = rd_u32(&p[44]);
    snapshot.vel_n_mms = rd_i32(&p[48]);
    snapshot.vel_e_mms = rd_i32(&p[52]);
    snapshot.vel_d_mms = rd_i32(&p[56]);
    snapshot.gspeed_mms = rd_i32(&p[60]);
    snapshot.head_mot_e5 = rd_i32(&p[64]);
    if (payload_len > 76) snapshot.num_sv = p[76];
    if (payload_len > 78) snapshot.valid_fix = (p[78] & 0x01) != 0;
    else snapshot.valid_fix = (snapshot.fix_type == UBX_FIX_2D || snapshot.fix_type == UBX_FIX_3D);
    snapshot.last_pvt_us = micros();
}

static void parse_nav_sat(const uint8_t *p, uint16_t len) {
    if (!p || len < 8) return;
    snapshot.num_sv = p[5];
    snapshot.last_sat_us = micros();
}

void ubx_init(void) {
    state = UBX_WAIT_SYNC1;
    payload_idx = 0;
    payload_len = 0;
    msg_class = msg_id = 0;
    ck_a = ck_b = exp_ck_a = exp_ck_b = 0;
    memset(&snapshot, 0, sizeof(snapshot));
    ubx_configure_messages();
}

void ubx_configure_messages(void) {
    uint8_t cfg_msg[8];
    cfg_msg[0] = UBX_CLASS_NAV;
    cfg_msg[1] = UBX_ID_PVT;
    cfg_msg[2] = 1;
    cfg_msg[3] = 1;
    cfg_msg[4] = 1;
    cfg_msg[5] = 1;
    cfg_msg[6] = 1;
    cfg_msg[7] = 1;
    (void)cfg_msg;
    /* Will send over UART2 if available */
}

void ubx_process(void) {
    uint8_t b;
    while (uart_getc(USART2, &b)) {
        switch (state) {
        case UBX_WAIT_SYNC1:
            if (b == UBX_SYNC1) state = UBX_WAIT_SYNC2;
            break;
        case UBX_WAIT_SYNC2:
            if (b == UBX_SYNC2) state = UBX_WAIT_CLASS;
            else if (b == UBX_SYNC1) state = UBX_WAIT_SYNC2;
            else state = UBX_WAIT_SYNC1;
            break;
        case UBX_WAIT_CLASS:
            msg_class = b; fletcher_reset(); fletcher_add(b); state = UBX_WAIT_ID;
            break;
        case UBX_WAIT_ID:
            msg_id = b; fletcher_add(b); state = UBX_WAIT_LEN_LO;
            break;
        case UBX_WAIT_LEN_LO:
            payload_len = b; fletcher_add(b); state = UBX_WAIT_LEN_HI;
            break;
        case UBX_WAIT_LEN_HI:
            payload_len |= ((uint16_t)b << 8); fletcher_add(b);
            payload_idx = 0;
            if (payload_len > sizeof(payload)) payload_len = sizeof(payload);
            state = (payload_len == 0) ? UBX_WAIT_CK_A : UBX_WAIT_PAYLOAD;
            break;
        case UBX_WAIT_PAYLOAD:
            if (payload_idx < payload_len) {
                payload[payload_idx++] = b;
                fletcher_add(b);
            }
            if (payload_idx >= payload_len) state = UBX_WAIT_CK_A;
            break;
        case UBX_WAIT_CK_A:
            exp_ck_a = b;
            state = UBX_WAIT_CK_B;
            break;
        case UBX_WAIT_CK_B:
            exp_ck_b = b;
            if (exp_ck_a == ck_a && exp_ck_b == ck_b) {
                if (msg_class == UBX_CLASS_NAV && msg_id == UBX_ID_PVT) {
                    parse_nav_pvt(payload);
                } else if (msg_class == UBX_CLASS_NAV && msg_id == UBX_ID_SAT) {
                    parse_nav_sat(payload, payload_len);
                }
            }
            state = UBX_WAIT_SYNC1;
            break;
        default:
            state = UBX_WAIT_SYNC1;
            break;
        }
    }
}

const ubx_snapshot_t* ubx_get_snapshot(void) { return &snapshot; }

void failsafe_update_gps_from_ubx(void) {
    const ubx_snapshot_t *gps = ubx_get_snapshot();
    if (gps->last_pvt_us == 0) return;
    if ((micros() - gps->last_pvt_us) > 500000) return;
    if (!gps->valid_fix && gps->fix_type != UBX_FIX_2D && gps->fix_type != UBX_FIX_3D) return;
    failsafeUpdateGps(gps->lat_e7 * 1e-7, gps->lon_e7 * 1e-7, gps->alt_msl_mm / 1000.0f, gps->num_sv, gps->gspeed_mms / 1000.0f);
}
