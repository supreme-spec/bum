#include "radun.h"
#include <string.h>

#define MSP_IDENT 100
#define MSP_PID 112
#define MSP_SET_PID 202

static uint8_t msp_buf[256];

static void msp_reply(uint8_t cmd, const uint8_t *data, uint8_t len) {
    /* Placeholder: send via USART6 */
    (void)cmd; (void)data; (void)len;
}

void msp_process(uint8_t cmd, uint8_t *payload, uint8_t len) {
    switch (cmd) {
        case MSP_IDENT: {
            uint8_t id[7] = {'R','A','D','U','N',1,0};
            msp_reply(MSP_IDENT, id, 7);
            break;
        }
        case MSP_PID: {
            /* send 3*4 floats as bytes not implemented fully */
            msp_reply(MSP_PID, msp_buf, 0);
            break;
        }
        case MSP_SET_PID: {
            if (len >= 12) {
                memcpy(msp_buf, payload, 12);
            }
            break;
        }
        default:
            break;
    }
}