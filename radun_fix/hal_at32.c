/* ============================================================================
 * hal_at32.c — замена hal.c. Содержит четыре исправления, без которых плата
 *              не запускается ВООБЩЕ (см. RADUN_AUDIT.md, раздел P0).
 *
 *  1) включение FPU (CPACR) — иначе первая же float-инструкция даёт UsageFault;
 *  2) корректная настройка PLL/делителей и FLASH wait-states;
 *  3) gpio_af() через MUXL/MUXH (в AT32 нет AFSEL в GPIO);
 *  4) uart_init() через делитель BAUD = PCLK/baud (а не PCLK/(16*baud)).
 * ============================================================================ */
#include "radun.h"

uint32_t g_sclk_hz  = 288000000UL;
uint32_t g_hclk_hz  = 288000000UL;
uint32_t g_pclk1_hz = 144000000UL;
uint32_t g_pclk2_hz = 144000000UL;

/* ------------------------------------------------------------------------- */
/*  ВЫБОР ТАКТИРОВАНИЯ                                                        */
/*  Для первого включения платы настоятельно рекомендую 0 (HICK 48 МГц):      */
/*  так вы отладите GPIO/SPI/UART, не завися от кварца и PLL.                 */
/* ------------------------------------------------------------------------- */
#ifndef CLOCK_USE_PLL
#define CLOCK_USE_PLL 0
#endif

#define HEXT_HZ        8000000UL
#define HICK_48MHZ     48000000UL

#ifndef PLL_FR_288
#define PLL_FR_288     4UL
#endif

#ifndef FLASH_WS
#define FLASH_WS       6UL
#endif

static volatile uint32_t s_ms;
static volatile uint32_t s_cyc_at_tick;

static void __attribute__((unused)) clock_fallback_hick(void)
{
    CRM_CTRL |= CRM_CTRL_HICKEN;
    while (!(CRM_CTRL & CRM_CTRL_HICKSTBL)) { }
    CRM_CFG = (CRM_CFG & ~0xFUL) | CRM_CFG_SCLKSEL(0);
    while ((CRM_CFG & CRM_CFG_SCLKSTS_MASK) != 0U) { }
    g_sclk_hz = g_hclk_hz = 48000000UL;
    g_pclk1_hz = g_pclk2_hz = 48000000UL;
}

void SystemInit(void)
{
    /* --- 1. FPU: без этого весь float-код падает в UsageFault --------------- */
    CORE_SCB_CPACR |= (0xFUL << 20);
    __asm volatile ("dsb");
    __asm volatile ("isb");

    /* --- 2. FLASH wait states ---------------------------------------------- */
    FLASH_ACR = FLASH_WS;

    /* --- 3. Делители: AHB=/1, APB1=/2, APB2=/2 ----------------------------- */
    uint32_t cfg = CRM_CFG;
    cfg &= ~(0xFUL << 4);   cfg |= CRM_CFG_AHBDIV(CRM_DIV_VAL1);
    cfg &= ~(0x7UL << 8);   cfg |= CRM_CFG_APB1DIV(CRM_DIV_VAL2);
    cfg &= ~(0x7UL << 11);  cfg |= CRM_CFG_APB2DIV(CRM_DIV_VAL2);

#if CLOCK_USE_PLL
    /* --- 4. HEXT 8 МГц + PLL 288 МГц ------------------------------------- */
    CRM_CTRL |= CRM_CTRL_HEXTEN;
    {
        uint32_t to = 2000000U;
        while (!(CRM_CTRL & CRM_CTRL_HEXTSTBL) && --to) { }
        if (to == 0U) { clock_fallback_hick(); return; }
    }

    CRM_PLLCFG = CRM_PLL_PLLMS(1) | CRM_PLL_PLLNS(36) | CRM_PLL_PLLFR(PLL_FR_288);
    CRM_CTRL |= CRM_CTRL_PLLEN;
    {
        uint32_t to = 2000000U;
        while (!(CRM_CTRL & CRM_CTRL_PLLSTBL) && --to) { }
        if (to == 0U) { clock_fallback_hick(); return; }
    }
    cfg &= ~3UL; cfg |= CRM_CFG_SCLKSEL(2);
    CRM_CFG = cfg;
    while ((CRM_CFG & CRM_CFG_SCLKSTS_MASK) != (2UL << 2)) { }
    g_sclk_hz = 288000000UL;
    g_hclk_hz = g_sclk_hz;
    g_pclk1_hz = g_hclk_hz / 2U;
    g_pclk2_hz = g_hclk_hz / 2U;
#else
    CRM_CTRL |= CRM_CTRL_HICKEN;
    while (!(CRM_CTRL & CRM_CTRL_HICKSTBL)) { }
    cfg &= ~3UL; cfg |= CRM_CFG_SCLKSEL(0);
    CRM_CFG = cfg;
    while ((CRM_CFG & CRM_CFG_SCLKSTS_MASK) != 0U) { }
    g_sclk_hz = g_hclk_hz = HICK_48MHZ;
    g_pclk1_hz = g_pclk2_hz = HICK_48MHZ;
#endif

    /* --- 5. DWT для микросекунд + SysTick 1 кГц --------------------------- */
    CORE_DEMCR      |= (1UL << 24);
    DWT->CTRL       |= DWT_CTRL_CYCCNTENA;
    DWT->CYCCNT      = 0U;

    SYSTICK->LOAD    = (g_sclk_hz / 1000U) - 1U;
    SYSTICK->VAL     = 0U;
    SYSTICK->CTRL    = 7U;
}

void SysTick_Handler(void)
{
    s_cyc_at_tick = DWT->CYCCNT;
    s_ms++;
}

uint32_t hal_ms(void) { return s_ms; }

void delay_us(uint32_t us)
{
    uint32_t start = DWT->CYCCNT;
    uint32_t ticks = us * (g_sclk_hz / 1000000U);
    while ((DWT->CYCCNT - start) < ticks) { }
}

void delay_ms(uint32_t ms)
{
    uint32_t start = s_ms;
    while ((s_ms - start) < ms) { }
}

uint32_t micros(void)
{
    uint32_t cyc = DWT->CYCCNT;
    uint32_t cyc0 = s_cyc_at_tick;
    uint32_t ms = s_ms;
    return ms * 1000U + ((cyc - cyc0) / (g_sclk_hz / 1000000U));
}

/* ---------------------------------------------------------------------------- */
/*  GPIO                                                                        */
/* ---------------------------------------------------------------------------- */
void gpio_out(GPIO_t *p, uint16_t pin)
{
    uint8_t n = (uint8_t)__builtin_ctz(pin);
    p->CFGR &= ~(3UL << (n * 2U));
    p->CFGR |=  (1UL << (n * 2U));
    p->OMODE &= ~(1UL << n);
    p->PULL  &= ~(3UL << (n * 2U));
}

void gpio_af(GPIO_t *p, uint16_t pin, uint8_t af)
{
    uint8_t n = (uint8_t)__builtin_ctz(pin);
    if (n < 8U) {
        uint32_t sh = (uint32_t)n * 4U;
        p->MUXL = (p->MUXL & ~(0xFUL << sh)) | ((uint32_t)(af & 0xFU) << sh);
    } else {
        uint32_t sh = ((uint32_t)n - 8U) * 4U;
        p->MUXH = (p->MUXH & ~(0xFUL << sh)) | ((uint32_t)(af & 0xFU) << sh);
    }
    p->CFGR &= ~(3UL << (n * 2U));
    p->CFGR |=  (2UL << (n * 2U));
    p->OMODE &= ~(1UL << n);
}

void gpio_set(GPIO_t *p, uint16_t pin) { p->SCR = pin; }
void gpio_clr(GPIO_t *p, uint16_t pin) { p->CLR = pin; }
bool gpio_get(GPIO_t *p, uint16_t pin) { return (p->IDT & pin) != 0U; }

/* ---------------------------------------------------------------------------- */
/*  USART                                                                       */
/* ---------------------------------------------------------------------------- */
void uart_init(USART_t *u, uint32_t pclk_hz, uint32_t baud)
{
    u->CTRL1 = 0U;
    u->BAUD  = (pclk_hz + baud / 2U) / baud;
    u->CTRL1 = USART_CTRL1_REN | USART_CTRL1_TEN | USART_CTRL1_UEN;
}

void uart_putc(USART_t *u, uint8_t b)
{
    uint32_t to = 1000000U;
    while (!(u->STS & USART_STS_TDBE)) { if (--to == 0U) return; }
    u->DT = b;
}

bool uart_getc(USART_t *u, uint8_t *b)
{
    if (u->STS & USART_STS_RDBF) { *b = (uint8_t)u->DT; return true; }
    return false;
}

/* ---------------------------------------------------------------------------- */
/*  SPI                                                                         */
/* ---------------------------------------------------------------------------- */
void spi_init(SPI_t *s)
{
    s->CTRL1 = 0U;
    s->CTRL1 = (1UL << 9) | (1UL << 8) | (4UL << 3) | (1UL << 6);
}

uint8_t spi_rw(SPI_t *s, uint8_t tx)
{
    uint32_t to = 100000U;
    while (!(s->STS & (1UL << 1))) { if (--to == 0U) return 0xFFU; }
    s->DT = tx;
    to = 100000U;
    while (!(s->STS & (1UL << 0))) { if (--to == 0U) return 0xFFU; }
    return (uint8_t)s->DT;
}

/* ---------------------------------------------------------------------------- */
/*  Прочее                                                                      */
/* ---------------------------------------------------------------------------- */
void NVIC_SetPriority(uint8_t irqn, uint32_t prio)
{
    CORE_NVIC_IPR(irqn) = (uint8_t)((prio << 4) & 0xF0U);
}

void hal_init(void)
{
    CRM_AHB1EN |= AHB1EN_GPIO(0) | AHB1EN_GPIO(1) | AHB1EN_GPIO(2);
    CRM_APB1EN |= APB1EN_TMR6 | APB1EN_TMR7 | APB1EN_SPI2 | APB1EN_I2C1;
    CRM_APB2EN |= APB2EN_TMR1 | APB2EN_TMR8 | APB2EN_SPI1 | APB2EN_ADC1 | APB2EN_USART6;
}

void NVIC_SystemReset(void)
{
    __asm volatile ("dsb");
    CORE_SCB_AIRCR = 0x05FA0004UL;
    while (1) { __asm volatile ("nop"); }
}

/* ---------------------------------------------------------------------------- */
/*  ADC + DMA (два канала, циклически, без блокировок в ISR)                   */
/* ---------------------------------------------------------------------------- */
#define ADC_DMA_BUF_LEN 64
static uint16_t adc_dma_buf[ADC_DMA_BUF_LEN];
static volatile uint16_t adc_vbat_raw;
static volatile uint16_t adc_curr_raw;

void adc_dma_init(void)
{
    CRM_AHB1EN |= AHB1EN_DMA1;
    CRM_APB2EN |= APB2EN_ADC1;

    DMA1_CHANNEL1->CCR = 0U;
    DMA1_CHANNEL1->CPAR = (uint32_t)&ADC1->RDAT;
    DMA1_CHANNEL1->CM1AR = (uint32_t)adc_dma_buf;
    DMA1_CHANNEL1->CNDTR = ADC_DMA_BUF_LEN;
    DMA1_CHANNEL1->CCR = DMA_CCR_EN | DMA_CCR_TCIE | DMA_CCR_CIRC |
                         DMA_CCR_MINC | DMA_CCR_PSIZE_16 | DMA_CCR_MSIZE_16 |
                         DMA_CCR_PL_HIGH;

    ADC1->CTRL2 |= ADC_CTRL2_ADCON;
    ADC1->CTRL2 |= (1UL << 8);         /* DMAEN (сверять с RM AT32F435) */
}

void adc_dma_update(void)
{
    uint32_t cnt = ADC_DMA_BUF_LEN;
    uint32_t vsum = 0, isum = 0, n = 0;
    for (uint32_t i = 0; i < cnt; i += 2) {
        vsum += adc_dma_buf[i];
        isum += adc_dma_buf[i + 1];
        n++;
    }
    if (n > 0) {
        adc_vbat_raw = (uint16_t)(vsum / n);
        adc_curr_raw = (uint16_t)(isum / n);
    }
}

uint16_t adc_get_vbat_raw(void) { return adc_vbat_raw; }
uint16_t adc_get_curr_raw(void) { return adc_curr_raw; }
