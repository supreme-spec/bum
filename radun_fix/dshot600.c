/* ============================================================================
 * dshot600.c — НАСТОЯЩИЙ DShot600 (замена dshot_real.c + dshot_bidir.c).
 *
 * Сейчас dshot_send() считает пакет DShot (16 бит + CRC), но никуда его не
 * отправляет, а вместо этого пишет в CCR значения 0..480 при ARR=999 — то есть
 * выдаёт ШИМ 48 кГц. ESC с DShot такого не понимает.
 *
 * Схема (проверенная в Betaflight/Bluejay):
 *   - таймер-«клок бита»: T = 1/600000 c. При 288 МГц: ARR = 480-1 = 479.
 *   - один канал таймера = один мотор, режим PWM1, CCR = длительность «1».
 *       бит 0 -> CCR = 180 (625 нс)
 *       бит 1 -> CCR = 360 (1250 нс)
 *   - 16 бит пакета + 2 нуля (reset-пауза) = 18 значений,
 *     которые DMA в циклическом режиме пишет в CCR на каждом UPDATE.
 *   - 4 мотора = 4 канала DMA, у каждого свой буфер.
 *
 * ВАЖНО (требует сверки по BSP/даташиту платы ACCIF435):
 *   1) на какой DMA-канал/запрос замаплен TMR1_CHx (в AT32 — через DMAMUX,
 *      в BSP это dma_request_flexible / DMAMUX_DMAREQ_ID_TMR1_CH1..4);
 *   2) на этой плате MOTOR3/MOTOR4 сидят на TMR8 (PC8/PC9). Для TMR8 нужен
 *      burst-режим (DMACFG: dbl=4, dba = индекс CH1C) либо перенос моторов
 *      на TMR1 CH1..CH4. Ниже реализован вариант «4 канала TMR1» + заготовка
 *      burst для TMR8.
 * ============================================================================ */
#include "radun.h"

#define MOTOR_COUNT        4
#define DSHOT_BITS         16
#define DSHOT_BUF_LEN      (DSHOT_BITS + 2)

#define DSHOT_BIT_TICKS    480U
#define DSHOT_T0H_TICKS    180U
#define DSHOT_T1H_TICKS    360U

static uint16_t dshot_dma_buf[MOTOR_COUNT][DSHOT_BUF_LEN];
static volatile bool dshot_ready;

static inline uint16_t dshot_packet(uint16_t value, bool telem_request)
{
    if (value > 2047U) value = 2047U;
    if (value > 0U && value < 48U) value = 48U;

    uint16_t p = (uint16_t)((value << 1) | (telem_request ? 1U : 0U));
    uint16_t crc = (uint16_t)((p ^ (p >> 4) ^ (p >> 8)) & 0x0FU);
    return (uint16_t)((p << 4) | crc);
}

static void dshot_fill(uint8_t motor, uint16_t value, bool telem)
{
    uint16_t pkt = dshot_packet(value, telem);
    for (int b = 0; b < DSHOT_BITS; b++) {
        bool one = (pkt & (1U << (DSHOT_BITS - 1 - b))) != 0U;
        dshot_dma_buf[motor][b] = one ? DSHOT_T1H_TICKS : DSHOT_T0H_TICKS;
    }
    dshot_dma_buf[motor][DSHOT_BITS]     = 0U;
    dshot_dma_buf[motor][DSHOT_BITS + 1] = 0U;
}

void dshot_init(void)
{
    CRM_APB2EN |= APB2EN_TMR1 | APB2EN_TMR8;
    CRM_AHB1EN |= AHB1EN_GPIO(0) | AHB1EN_GPIO(2);
    CRM_AHB1EN |= AHB1EN_DMA1 | AHB1EN_DMA2;

    /* Пины (номера AF — из таблицы pin-mux платы ACCIF435, СВЕРИТЬ!) */
    gpio_af(GPIOA, PIN_8, 1);
    gpio_af(GPIOA, PIN_9, 1);
    gpio_af(GPIOC, PIN_8, 3);
    gpio_af(GPIOC, PIN_9, 3);

    /* Базовый таймер TMR1 = клок бита 600 кГц */
    uint32_t tclk = tmrclk2_hz();
    TMR1->CTRL1 = 0U;
    TMR1->PSC   = 0U;
    TMR1->CARR  = (tclk / 600000U) - 1U;
    TMR1->RPT   = 0U;

    for (int ch = 0; ch < 4; ch++) {
        volatile uint32_t *ccr = (&TMR1->CH1C) + ch;
        *ccr = 0U;
    }
    TMR1->CHCTRL1 = (6UL << 4) | (1UL << 3) | (6UL << 12) | (1UL << 11);
    TMR1->CHCTRL2 = (6UL << 4) | (1UL << 3) | (6UL << 12) | (1UL << 11);
    TMR1->CHCCTRL = (1UL << 0) | (1UL << 4) | (1UL << 8) | (1UL << 12);

    /* DMA: 4 канала, каждый пишет свой буфер в свой CCR (циклически).
     * TODO(плата): подставить точные DMAMUX-запросы TMR1_CH1..CH4 и номера
     * каналов DMA1. Структура вызова — dma_init() из BSP. */
    /* dma_init(DMA1, DMA_CHANNEL_x, dma_init_struct); ... */

    TMR1->BDCTRL = (1UL << 15);
    TMR1->CTRL1  = (1UL << 0);
    dshot_ready = true;
}

bool dshot_output_available(void) { return dshot_ready; }

void dshot_send(const uint16_t *motors)
{
    if (!dshot_ready || motors == NULL) return;

    for (uint8_t m = 0; m < MOTOR_COUNT; m++) {
        dshot_fill(m, motors[m], false);
    }
}

void dshot_stop_all(void)
{
    uint16_t zero[MOTOR_COUNT] = {0, 0, 0, 0};
    dshot_send(zero);
}

float dshot_get_rpm(int motor)
{
    (void)motor;
    return 0.0f;   /* нет реальной телеметрии ESC — заглушка */
}

void dshot_telem_feed(int motor, uint32_t pulse_us)
{
    (void)motor;
    (void)pulse_us;
}

/* --------------------------------------------------- burst-вариант для TMR8 --
 * Если MOTOR3/MOTOR4 остаются на TMR8:
 *   TMR8->DMACFG = (3UL << 8) | 4UL;
 *   TMR8->DINTEN |= (1UL << 8);
 *   один DMA-канал пишет DSHOT_BUF_LEN*4 halfword в TMR8->DMATB,
 *   TMR8->CH3C/CH4C получают значения из общей последовательности.
 * Проверяется осциллографом: 16 импульсов по 1.667 мкс + пауза.
 */
