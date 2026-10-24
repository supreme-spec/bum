#include "radun.h"
#include <string.h>

#define MSP_IDENT 100
#define MSP_PID 112
#define MSP_SET_PID 202
#define MSP_STATUS 101
#define MSP_RC 105

static uint8_t msp_buf[256];

static uint8_t msp_crc8(const uint8_t *data, uint8_t len) {
    uint8_t crc = 0;
    for (uint8_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x80) crc = (uint8_t)((crc << 1) ^ 0xD5);
            else crc = (uint8_t)(crc << 1);
        }
    }
    return crc;
}

static void msp_reply(uint8_t cmd, const uint8_t *data, uint8_t len) {
    uint8_t frame[256];
    uint8_t idx = 0;

    frame[idx++] = '$';
    frame[idx++] = 'M';
    frame[idx++] = '>';
    frame[idx++] = len;
    frame[idx++] = cmd;

    if (len > 0 && data != NULL) {
        memcpy(&frame[idx], data, len);
        idx += len;
    }

    uint8_t crc = msp_crc8(&frame[3], len + 1);
    frame[idx++] = crc;

    /* Use USB CDC if connected, otherwise fallback to USART6 */
    if (usb_cdc_is_connected()) {
        usb_cdc_send(frame, idx);
    } else {
        for (uint8_t i = 0; i < idx; i++) {
            uart_putc(USART6, frame[i]);
        }
    }
}

void msp_process(uint8_t cmd, uint8_t *payload, uint8_t len) {
    extern pid_t pid;
    extern crsf_t rx;
    extern bat_t bat;
    extern imu_t imu;
    extern bool armed;
    extern bool fs_active;

    switch (cmd) {
        case MSP_IDENT: {
            uint8_t id[7] = {'R','A','D','U','N',1,0};
            msp_reply(MSP_IDENT, id, 7);
            break;
        }
        case MSP_PID: {
            uint8_t pid_data[36];
            memcpy(&pid_data[0], &pid.kp[0], 4);
            memcpy(&pid_data[4], &pid.ki[0], 4);
            memcpy(&pid_data[8], &pid.kd[0], 4);
            memcpy(&pid_data[12], &pid.kp[1], 4);
            memcpy(&pid_data[16], &pid.ki[1], 4);
            memcpy(&pid_data[20], &pid.kd[1], 4);
            memcpy(&pid_data[24], &pid.kp[2], 4);
            memcpy(&pid_data[28], &pid.ki[2], 4);
            memcpy(&pid_data[32], &pid.kd[2], 4);
            msp_reply(MSP_PID, pid_data, 36);
            break;
        }
        case MSP_SET_PID: {
            if (len >= 36) {
                memcpy(&pid.kp[0], &payload[0], 4);
                memcpy(&pid.ki[0], &payload[4], 4);
                memcpy(&pid.kd[0], &payload[8], 4);
                memcpy(&pid.kp[1], &payload[12], 4);
                memcpy(&pid.ki[1], &payload[16], 4);
                memcpy(&pid.kd[1], &payload[20], 4);
                memcpy(&pid.kp[2], &payload[24], 4);
                memcpy(&pid.ki[2], &payload[28], 4);
                memcpy(&pid.kd[2], &payload[32], 4);
            }
            break;
        }
        case MSP_STATUS: {
            uint8_t status[11];
            status[0] = armed ? 1 : 0;
            status[1] = fs_active ? 1 : 0;
            status[2] = rx.valid ? 1 : 0;
            memcpy(&status[3], &bat.v, 4);
            status[7] = (uint8_t)(rx.thr >> 8);
            status[8] = (uint8_t)rx.thr;
            status[9] = imu.healthy ? 1 : 0;
            status[10] = 0;
            msp_reply(MSP_STATUS, status, 11);
            break;
        }
        case MSP_RC: {
            uint16_t rc[8];
            rc[0] = rx.thr;
            rc[1] = rx.r + 1500;
            rc[2] = rx.p + 1500;
            rc[3] = rx.y + 1500;
            rc[4] = (rx.aux & 1) ? 2000 : 1000;
            rc[5] = (rx.aux & 2) ? 2000 : 1000;
            rc[6] = (rx.aux & 4) ? 2000 : 1000;
            rc[7] = (rx.aux & 8) ? 2000 : 1000;
            msp_reply(MSP_RC, (uint8_t*)rc, 16);
            break;
        }
        default:
            break;
    }
}