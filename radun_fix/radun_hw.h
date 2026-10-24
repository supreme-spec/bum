/* ============================================================================
 * radun_hw.h — КОРРЕКТНАЯ карта регистров AT32F435 (замена для radun.h,
 *              который написан по образцу STM32F4 и НЕ совпадает с AT32).
 *
 * Проверено по AT32F435/437 Firmware Library (v2.0.9) и RM_AT32F435_437:
 *   CRM : ctrl 0x00, pllcfg 0x04, cfg 0x08, clkint 0x0C,
 *         ahb1rst 0x10, ahb2rst 0x14, ahb3rst 0x18,
 *         apb1rst 0x20, apb2rst 0x24,
 *         ahb1en 0x30, ahb2en 0x34, ahb3en 0x38,
 *         apb1en 0x40, apb2en 0x44
 *   GPIO: cfgr 0x00, omode 0x04, odrvr 0x08, pull 0x0C, idt 0x10, odt 0x14,
 *         scr 0x18, wpen 0x1C, muxl 0x20, muxh 0x24, clr 0x28, hdrv 0x3C
 *   USART: sts 0x00, dt 0x04, baudr 0x08 (16-бит DIV = PCLK/baud),
 *          ctrl1 0x0C, ctrl2 0x10, ctrl3 0x14
 *
 * Ключевые отличия от того, что было в проекте:
 *   - НЕТ регистра AFSEL в GPIO: альтернативная функция выбирается в MUXL/MUXH,
 *     а биты CFGR только переключают пин в режим «mux» (0b10);
 *   - BAUD — это делитель, а не BRR по formule STM32 (PCLK/(16*baud));
 *   - включение тактирования — APB1EN/APB2EN по ОТДЕЛЬНЫМ адресам (0x40/0x44),
 *     а не «APB1=...» по смещению 0x0C (там clkint!);
 *   - тактирование таймеров = PCLK * 2, если делитель APBx > 1.
 * ============================================================================ */
#pragma once
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#define REG32(a)        (*(volatile uint32_t *)(a))
#define REG16(a)        (*(volatile uint16_t *)(a))
#define REG8(a)         (*(volatile uint8_t  *)(a))

/* ---------------------------------------------------------------- базовые --- */
#define PERIPH_BASE     0x40000000UL
#define APB1_BASE       (PERIPH_BASE)
#define APB2_BASE       (PERIPH_BASE + 0x00010000UL)
#define AHB1_BASE       (PERIPH_BASE + 0x00020000UL)

/* ------------------------------------------------------------------- CRM ---- */
#define CRM             (AHB1_BASE + 0x3800UL)
#define CRM_CTRL        REG32(CRM + 0x00)
#define CRM_PLLCFG      REG32(CRM + 0x04)
#define CRM_CFG         REG32(CRM + 0x08)
#define CRM_CLKINT      REG32(CRM + 0x0C)
#define CRM_AHB1RST     REG32(CRM + 0x10)
#define CRM_AHB2RST     REG32(CRM + 0x14)
#define CRM_AHB3RST     REG32(CRM + 0x18)
#define CRM_APB1RST     REG32(CRM + 0x20)
#define CRM_APB2RST     REG32(CRM + 0x24)
#define CRM_AHB1EN      REG32(CRM + 0x30)
#define CRM_AHB2EN      REG32(CRM + 0x34)
#define CRM_AHB3EN      REG32(CRM + 0x38)
#define CRM_APB1EN      REG32(CRM + 0x40)
#define CRM_APB2EN      REG32(CRM + 0x44)

/* CTRL */
#define CRM_CTRL_HICKEN         (1UL << 0)
#define CRM_CTRL_HICKSTBL       (1UL << 1)
#define CRM_CTRL_HEXTEN         (1UL << 16)
#define CRM_CTRL_HEXTSTBL       (1UL << 17)
#define CRM_CTRL_HEXTBYPS       (1UL << 18)
#define CRM_CTRL_PLLEN          (1UL << 24)
#define CRM_CTRL_PLLSTBL        (1UL << 25)

/* CFG: sclksel[1:0], sclksts[3:2], ahbdiv[7:4], apb1div[10:8], apb2div[13:11] */
#define CRM_CFG_SCLKSEL(n)      ((uint32_t)(n) & 3UL)
#define CRM_CFG_SCLKSTS_MASK    (3UL << 2)
#define CRM_CFG_AHBDIV(n)       (((uint32_t)(n) & 0xFUL) << 4)
#define CRM_CFG_APB1DIV(n)      (((uint32_t)(n) & 0x7UL) << 8)
#define CRM_CFG_APB2DIV(n)      (((uint32_t)(n) & 0x7UL) << 11)

#define CRM_DIV_VAL1            0UL
#define CRM_DIV_VAL2            4UL
#define CRM_DIV_VAL4            5UL
#define CRM_DIV_VAL8            6UL
#define CRM_DIV_VAL16           7UL

/* PLLCFG: pllms[3:0], pllns[14:6], pllfr[18:16], pllrcs[22] */
#define CRM_PLL_PLLMS(n)        ((uint32_t)(n) & 0xFUL)
#define CRM_PLL_PLLNS(n)        ((((uint32_t)(n)) & 0x1FFUL) << 6UL)
#define CRM_PLL_PLLFR(n)        ((((uint32_t)(n)) & 0x7UL) << 16UL)
#define CRM_PLL_RCS             (1UL << 22)

/* CRM_APB1EN биты */
#define APB1EN_TMR2   (1UL << 0)
#define APB1EN_TMR3   (1UL << 1)
#define APB1EN_TMR4   (1UL << 2)
#define APB1EN_TMR5   (1UL << 3)
#define APB1EN_TMR6   (1UL << 4)
#define APB1EN_TMR7   (1UL << 5)
#define APB1EN_WWDT   (1UL << 11)
#define APB1EN_SPI2   (1UL << 14)
#define APB1EN_USART2 (1UL << 17)
#define APB1EN_USART3 (1UL << 18)
#define APB1EN_I2C1   (1UL << 21)

/* CRM_APB2EN биты */
#define APB2EN_TMR1   (1UL << 0)
#define APB2EN_TMR8   (1UL << 1)
#define APB2EN_USART1 (1UL << 4)
#define APB2EN_USART6 (1UL << 5)
#define APB2EN_ADC1   (1UL << 8)
#define APB2EN_SPI1   (1UL << 12)

/* CRM_AHB1EN: порты GPIO + DMA */
#define AHB1EN_GPIO(n) (1UL << (n))
#define AHB1EN_DMA1    (1UL << 22)
#define AHB1EN_DMA2    (1UL << 24)

/* ------------------------------------------------------------------ GPIO ---- */
typedef struct {
    volatile uint32_t CFGR, OMODE, ODRVR, PULL, IDT, ODT,
                      SCR, WPEN, MUXL, MUXH, CLR;
} GPIO_t;

#define GPIOA  ((GPIO_t *)(AHB1_BASE + 0x0000UL))
#define GPIOB  ((GPIO_t *)(AHB1_BASE + 0x0400UL))
#define GPIOC  ((GPIO_t *)(AHB1_BASE + 0x0800UL))

#define PIN_0  (1U << 0)
#define PIN_1  (1U << 1)
#define PIN_2  (1U << 2)
#define PIN_3  (1U << 3)
#define PIN_4  (1U << 4)
#define PIN_5  (1U << 5)
#define PIN_6  (1U << 6)
#define PIN_7  (1U << 7)
#define PIN_8  (1U << 8)
#define PIN_9  (1U << 9)
#define PIN_10 (1U << 10)
#define PIN_11 (1U << 11)
#define PIN_12 (1U << 12)
#define PIN_13 (1U << 13)
#define PIN_14 (1U << 14)
#define PIN_15 (1U << 15)

/* ---------------------------------------------------------------- USART ----- */
typedef struct {
    volatile uint32_t STS, DT, BAUD, CTRL1, CTRL2, CTRL3, GDIV;
} USART_t;

#define USART1  ((USART_t *)(APB2_BASE + 0x1000UL))
#define USART2  ((USART_t *)(APB1_BASE + 0x4400UL))
#define USART3  ((USART_t *)(APB1_BASE + 0x4800UL))
#define USART6  ((USART_t *)(APB2_BASE + 0x1400UL))

#define USART_STS_RDBF     (1UL << 5)
#define USART_STS_TDC      (1UL << 6)
#define USART_STS_TDBE     (1UL << 7)
#define USART_CTRL1_REN    (1UL << 2)
#define USART_CTRL1_TEN    (1UL << 3)
#define USART_CTRL1_UEN    (1UL << 13)

/* ------------------------------------------------------------------------- */
/*  Таймер/SPI/ADC/IWDG — тактирование через CRM_APB1EN / CRM_APB2EN.        */
/* ------------------------------------------------------------------------- */
typedef struct {
    volatile uint32_t CTRL1, CTRL2, SMCTRL, DINTEN, INTF, SWEV,
                      CHCTRL1, CHCTRL2, CHCCTRL, CNT, PSC, CARR, RPT,
                      CH1C, CH2C, CH3C, CH4C, BDCTRL, DMACFG, DMATB;
} TMR_t;

#define TMR1  ((TMR_t *)(APB2_BASE + 0x0000UL))
#define TMR8  ((TMR_t *)(APB2_BASE + 0x3400UL))
#define TMR3  ((TMR_t *)(APB1_BASE + 0x0400UL))
#define TMR6  ((TMR_t *)(APB1_BASE + 0x1000UL))
#define TMR7  ((TMR_t *)(APB1_BASE + 0x1400UL))

typedef struct {
    volatile uint32_t CTRL1, CTRL2, STS, DT, _r[4];
} SPI_t;

#define SPI1  ((SPI_t *)(APB2_BASE + 0x3000UL))
#define SPI2  ((SPI_t *)(APB1_BASE + 0x3800UL))

typedef struct { volatile uint32_t CTRL1, CTRL2, OAR1, _r, DR, STS1, STS2; } I2C_t;
#define I2C1  ((I2C_t *)(APB1_BASE + 0x5400UL))

typedef struct {
    volatile uint32_t STS, CTRL1, CTRL2, SMPT1, SMPT2, _r[6],
                      RSQ1, RSQ2, RSQ3, _r2[4], RDAT;
} ADC_t;

#define ADC1      ((ADC_t *)(APB2_BASE + 0x2000UL))
#define ADC_CTRL2_ADCON  (1UL << 0)
#define ADC_CTRL2_CCEN   (1UL << 1)
#define ADC_CTRL2_OCEN   (1UL << 2)
#define ADC_CTRL2_RSWRC  (1UL << 22)
#define ADC_STS_CCE      (1UL << 1)

typedef struct {
    volatile uint32_t KR, PR, RLR, SR;
} IWDG_t;

#define IWDG  ((IWDG_t *)0x40003000UL)

#define IWDG_KEY_ENABLE 0xCCCCU
#define IWDG_KEY_WRITE  0x5555U
#define IWDG_KEY_RELOAD 0xAAAAU

#define FLASH_ACR  REG32(0x40023C00UL)

/* ------------------------------------------------------------------ DMA ----- */
/* AT32F435: DMA1 на AHB1, base 0x40020000; канал stride 0x14               */
#define DMA1_BASE       (AHB1_BASE + 0x0000UL)
#define DMA1            ((DMA_TypeDef *)(DMA1_BASE))

typedef struct {
    volatile uint32_t CCR, CNDTR, CPAR, CM1AR;
} DMA_Channel_t;

typedef struct {
    DMA_Channel_t ch[7];
    volatile uint32_t ISR, IFCR;
} DMA_TypeDef;

#define DMA1_CHANNEL1  (&DMA1->ch[0])
#define DMA1_CHANNEL2  (&DMA1->ch[1])

/* Биты CCR */
#define DMA_CCR_EN      (1UL << 0)
#define DMA_CCR_TCIE    (1UL << 1)
#define DMA_CCR_DIR     (1UL << 4)   /* 0: periph->mem, 1: mem->periph */
#define DMA_CCR_CIRC    (1UL << 5)
#define DMA_CCR_PINC    (1UL << 6)
#define DMA_CCR_MINC    (1UL << 7)
#define DMA_CCR_PSIZE_8  (0UL << 8)
#define DMA_CCR_PSIZE_16 (1UL << 8)
#define DMA_CCR_MSIZE_8  (0UL << 10)
#define DMA_CCR_MSIZE_16 (1UL << 10)
#define DMA_CCR_PL_LOW   (0UL << 12)
#define DMA_CCR_PL_HIGH  (3UL << 12)

/* ISR / IFCR флаги */
#define DMA_ISR_TCIF1   (1UL << 0)
#define DMA_ISR_TCIF2   (1UL << 5)
#define DMA_IFCR_CTCIF1 (1UL << 0)
#define DMA_IFCR_CTCIF2 (1UL << 5)
typedef struct { volatile uint32_t CTRL, CYCCNT; } DWT_t;
#define DWT ((DWT_t *)0xE0001000UL)
#define DWT_CTRL_CYCCNTENA (1UL << 0)

#define CORE_SCB_CPACR  REG32(0xE000ED88UL)
#define CORE_DEMCR      REG32(0xE000EDFCUL)
#define CORE_NVIC_ISER1 REG32(0xE000E104UL)
#define CORE_NVIC_IPR(n) REG8(0xE000E400UL + (n))
#define CORE_SCB_AIRCR  REG32(0xE000ED0CUL)

typedef struct { volatile uint32_t CTRL, LOAD, VAL, CALIB; } SYSTICK_t;
#define SYSTICK ((SYSTICK_t *)0xE000E010UL)

/* ------------------------------------------------------------------------- */
/*  Тактирование: зафиксируйте один раз после SystemInit().                    */
/* ------------------------------------------------------------------------- */
extern uint32_t g_sclk_hz;
extern uint32_t g_hclk_hz;
extern uint32_t g_pclk1_hz;
extern uint32_t g_pclk2_hz;

static inline uint32_t tmrclk1_hz(void) { return g_pclk1_hz * 2U; }
static inline uint32_t tmrclk2_hz(void) { return g_pclk2_hz * 2U; }

/* ------------------------------------------------------------------------- */
/*  Прототипы HAL (реализация — hal_at32.c / dshot600.c).                      */
/* ------------------------------------------------------------------------- */
void     SystemInit(void);
uint32_t hal_ms(void);
uint32_t micros(void);
void     delay_ms(uint32_t ms);
void     delay_us(uint32_t us);

void     gpio_out(GPIO_t *p, uint16_t pin);
void     gpio_af(GPIO_t *p, uint16_t pin, uint8_t af);
void     gpio_set(GPIO_t *p, uint16_t pin);
void     gpio_clr(GPIO_t *p, uint16_t pin);
bool     gpio_get(GPIO_t *p, uint16_t pin);

void     uart_init(USART_t *u, uint32_t pclk_hz, uint32_t baud);
void     uart_putc(USART_t *u, uint8_t b);
bool     uart_getc(USART_t *u, uint8_t *b);

void     spi_init(SPI_t *s);
uint8_t  spi_rw(SPI_t *s, uint8_t tx);

void     NVIC_SetPriority(uint8_t irqn, uint32_t prio);
void     NVIC_SystemReset(void);

void     dshot_init(void);
void     dshot_send(const uint16_t *motors);
void     dshot_stop_all(void);
bool     dshot_output_available(void);

void     adc_dma_init(void);
void     adc_dma_update(void);
uint16_t adc_get_vbat_raw(void);
uint16_t adc_get_curr_raw(void);
