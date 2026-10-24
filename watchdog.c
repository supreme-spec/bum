#include "watchdog.h"
#include "radun.h"

#define IWDG_KR_ENABLE  0xCCCC
#define IWDG_KR_WRITE   0x5555
#define IWDG_KR_RELOAD  0xAAAA

static bool reset_by_iwdg = false;

void watchdog_init(uint32_t timeout_ms) {
    uint32_t reload = (((40000UL / 32UL) * timeout_ms) / 1000UL);
    if (reload == 0U) reload = 1U;
    if (reload > 0xFFFU) reload = 0xFFFU;

    IWDG->KR = IWDG_KR_ENABLE;
    IWDG->KR = IWDG_KR_WRITE;
    IWDG->PR = 3;
    IWDG->RLR = reload;
    IWDG->KR = IWDG_KR_RELOAD;

    reset_by_iwdg = false;
}

void watchdog_feed(void) {
    IWDG->KR = IWDG_KR_RELOAD;
}

bool watchdog_was_reset(void) {
    return reset_by_iwdg;
}
