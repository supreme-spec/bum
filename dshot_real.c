/* dshot_real.c — настоящий DShot через DMA на AT32F435
 *
 * DShot — это 16 последовательных импульсов.
 * Bit=1: duty 70% (значение 335 при CARR=479)
 * Bit=0: duty 35% (значение 167 при CARR=479)
 *
 * Используем Memory-to-Peripheral DMA: массив 16×uint16_t → регистр CHxC по Update event.
 */
#include "radun.h"
#include <string.h>

#define DSHOT_BIT_0   167   /* 35% of 479 */
#define DSHOT_BIT_1   335   /* 70% of 479 */
#define DSHOT_CMD_STOP 0
#define DSHOT_BITS     16

static const uint16_t gcr_table[16] = {
    0x1E, 0x19, 0x18, 0x16, 0x15, 0x13, 0x12, 0x0E,
    0x0D, 0x0C, 0x0A, 0x09, 0x06, 0x05, 0x03, 0x1C
};

/* DMA буферы: по 16 значений на каждый канал */
static uint16_t dshot_buf_tim1[DSHOT_BITS * 2] __attribute__((aligned(4))); /* CH1 + CH2 */
static uint16_t dshot_buf_tim3[DSHOT_BITS * 2] __attribute__((aligned(4))); /* CH3 + CH4 */

static uint16_t dshot_encode(uint16_t value, bool telem) {
    uint16_t c = value ^ (value >> 4) ^ (value >> 8);
    c &= 0x0F;
    uint16_t frame = (value << 4) | c;
    if (telem) frame |= 0x8000;
    uint16_t result = 0;
    for (int i = 0; i < 4; i++) {
        result <<= 5;
        result |= gcr_table[(frame >> (12 - i*4)) & 0x0F];
    }
    return result;
}

/* Заполнить буфер DShot одним значением: 16 бит × (CH_A, CH_B) */
static void dshot_fill_pair(uint16_t *buf, uint16_t val_a, uint16_t val_b) {
    uint16_t enc_a = dshot_encode(val_a, false);
    uint16_t enc_b = dshot_encode(val_b, false);

    for (int i = 0; i < DSHOT_BITS; i++) {
        buf[i * 2 + 0] = (enc_a & (1 << (15 - i))) ? DSHOT_BIT_1 : DSHOT_BIT_0;
        buf[i * 2 + 1] = (enc_b & (1 << (15 - i))) ? DSHOT_BIT_1 : DSHOT_BIT_0;
    }
}

void dshot_init_all(void) {
    RCC->APB2 |= 1;       /* TIM1 clock */
    RCC->APB1 |= (1 << 1); /* TIM3 clock */

    /* Включаем DMA clock (DMA1) */
    RCC->AHB1 |= (1 << 21);

    /* GPIO AF для моторов */
    gpio_af(GPIOA, GPIO_PIN_8, 1); /* TIM1 CH1 */
    gpio_af(GPIOA, GPIO_PIN_9, 1); /* TIM1 CH2 */
    gpio_af(GPIOC, GPIO_PIN_8, 2); /* TIM3 CH3 */
    gpio_af(GPIOC, GPIO_PIN_9, 2); /* TIM3 CH4 */

    TIM1->PSC = 23;
    TIM1->CARR = 19;
    TIM1->CTRL1 = (1 << 7) | (1 << 2) | (1 << 0); /* ARPE | URS | CEN */
    TIM1->CHCTRL1 = (0x6 << 4) | (0x6 << 12) | (1 << 3) | (1 << 11); /* PWM1, preload */
    TIM1->CHCCTRL = (1 << 0) | (1 << 4); /* CC1E, CC2E */
    TIM1->BDCTRL = (1 << 15); /* MOE */

    TIM1->DMACFG = (1 << 8); /* UDE (update DMA request enable) */

    TIM3->PSC = 23;
    TIM3->CARR = 19;
    TIM3->CTRL1 = (1 << 7) | (1 << 2) | (1 << 0);
    TIM3->CHCTRL2 = (0x6 << 4) | (0x6 << 12) | (1 << 3) | (1 << 11);
    TIM3->CHCCTRL = (1 << 8) | (1 << 12); /* CC3E, CC4E */
    TIM3->BDCTRL = (1 << 15);
    TIM3->DMACFG = (1 << 8);

    /* Инициализируем буферы командой STOP */
    dshot_fill_pair(dshot_buf_tim1, DSHOT_CMD_STOP, DSHOT_CMD_STOP);
    dshot_fill_pair(dshot_buf_tim3, DSHOT_CMD_STOP, DSHOT_CMD_STOP);

    /* Channel 5 → TIM1_CH1C (DMA1) */
    volatile uint32_t *dma_ch5_ctrl = (volatile uint32_t *)(0x40020000 + 0x58);
    volatile uint32_t *dma_ch5_ndtr = (volatile uint32_t *)(0x40020000 + 0x5C);
    volatile uint32_t *dma_ch5_par  = (volatile uint32_t *)(0x40020000 + 0x60);
    volatile uint32_t *dma_ch5_mar  = (volatile uint32_t *)(0x40020000 + 0x64);

    *dma_ch5_ctrl = 0;
    *dma_ch5_ndtr = DSHOT_BITS * 2;
    *dma_ch5_par  = (uint32_t)&TIM1->CH1C;
    *dma_ch5_mar  = (uint32_t)dshot_buf_tim1;
    *dma_ch5_ctrl = (1 << 7)  /* MINC */
                  | (1 << 5)  /* MSIZE = 01 (16-bit) */
                  | (0 << 4)  /* PSIZE = 00 (16-bit) */
                  | (1 << 3)  /* CIRC */
                  | (1 << 0); /* EN */

    /* Channel 6 → TIM3_CH3C */
    volatile uint32_t *dma_ch6_ctrl = (volatile uint32_t *)(0x40020000 + 0x6C);
    volatile uint32_t *dma_ch6_ndtr = (volatile uint32_t *)(0x40020000 + 0x70);
    volatile uint32_t *dma_ch6_par  = (volatile uint32_t *)(0x40020000 + 0x74);
    volatile uint32_t *dma_ch6_mar  = (volatile uint32_t *)(0x40020000 + 0x78);

    *dma_ch6_ctrl = 0;
    *dma_ch6_ndtr = DSHOT_BITS * 2;
    *dma_ch6_par  = (uint32_t)&TIM3->CH3C;
    *dma_ch6_mar  = (uint32_t)dshot_buf_tim3;
    *dma_ch6_ctrl = (1 << 7) | (1 << 5) | (1 << 3) | (1 << 0);
}

void dshot_send(uint16_t motors[4]) {
    uint16_t dshot[4];

    for (int i = 0; i < 4; i++) {
        if (motors[i] < 1000) {
            dshot[i] = DSHOT_CMD_STOP;
        } else {
            uint16_t v = (motors[i] - 1000) * 2 + 48;
            if (v > 2047) v = 2047;
            dshot[i] = v;
        }
    }

    dshot_fill_pair(dshot_buf_tim1, dshot[1], dshot[0]);
    dshot_fill_pair(dshot_buf_tim3, dshot[2], dshot[3]);
}
