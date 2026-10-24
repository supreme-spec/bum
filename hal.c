#include "radun.h"

static volatile uint32_t ms_cnt;

void SystemInit(void) {
    RCC->CTRL |= (1 << 16); while (!(RCC->CTRL & (1 << 17)));
    FLASH_ACR = 6;
    RCC->PLL = (1 << 22) | (71 << 8) | 1;
    RCC->CTRL |= (1 << 24); while (!(RCC->CTRL & (1 << 25)));
    RCC->CTRL = (RCC->CTRL & ~3) | 2;
    RCC->APB1 = 0x1400;
    RCC->APB2 = 0x0800;
    RCC->AHB1 = 0x80;
    CoreDebug->DEMCR |= (1 << 24); DWT->CTRL |= 1;
    SysTick->LOAD = (SYSCLK_HZ / 1000) - 1;
    SysTick->CTRL = 7;
}

void SysTick_Handler(void) { ms_cnt++; }
uint32_t hal_ms(void) { return ms_cnt; }
uint32_t micros(void) {
    return DWT->CYCCNT / 288U;
}
void delay_ms(uint32_t m) { uint32_t start = hal_ms(); while (hal_ms() - start < m); }
void delay_us(uint32_t u) { uint32_t t = DWT->CYCCNT + u * 288; while (DWT->CYCCNT < t); }

void hal_init(void) {
    RCC->AHB1 |= 0x07;
    RCC->APB2 |= (1 << 12) | (1 << 8) | (1 << 4);
    RCC->APB1 |= (1 << 14) | (1 << 21) | (1 << 17) | (1 << 18);
}

void gpio_out(GPIO_t* p, uint16_t pin) {
    uint8_t n = __builtin_ctz(pin);
    p->MODE &= ~(3 << (n*2)); p->MODE |= (1 << (n*2));
}
void gpio_af(GPIO_t* p, uint16_t pin, uint8_t af) {
    uint8_t n = __builtin_ctz(pin);
    uint32_t* afr = (n < 8) ? &p->AFSEL[0] : &p->AFSEL[1];
    uint8_t i = (n < 8) ? n : (n - 8);
    *afr &= ~(0xF << (i*4)); *afr |= ((uint32_t)af << (i*4));
    p->MODE &= ~(3 << (n*2)); p->MODE |= (2 << (n*2));
}
void gpio_set(GPIO_t* p, uint16_t pin) { p->BSET = pin; }
void gpio_clr(GPIO_t* p, uint16_t pin) { p->BCLR = pin; }

void spi_init(SPI_t* s) {
    s->CTRL1 = (1<<9)|(1<<8)|(4<<3)|(1<<6);
}
uint8_t spi_rw(SPI_t* s, uint8_t tx) {
    volatile uint32_t timeout = 100000U;
    while (!(s->STS & (1<<1))) { if (--timeout == 0) return 0xFF; }
    s->DT = tx;
    timeout = 100000U;
    while (!(s->STS & (1<<0))) { if (--timeout == 0) return 0xFF; }
    return (uint8_t)s->DT;
}

void i2c_write(uint8_t addr, uint8_t reg, uint8_t val) {
    while (I2C1->STS2 & (1<<1));
    I2C1->CTRL1 |= (1 << 8);
    while (!(I2C1->STS1 & (1<<0)));
    I2C1->DR = addr << 1;
    while (!(I2C1->STS1 & (1<<1)));
    (void)I2C1->STS2;
    I2C1->DR = reg;
    while (!(I2C1->STS1 & (1<<2)));
    I2C1->DR = val;
    while (!(I2C1->STS1 & (1<<2)));
    I2C1->CTRL1 |= (1 << 9);
}

uint8_t i2c_read(uint8_t addr, uint8_t reg) {
    while (I2C1->STS2 & (1<<1));
    I2C1->CTRL1 |= (1 << 8);
    while (!(I2C1->STS1 & (1<<0)));
    I2C1->DR = addr << 1;
    while (!(I2C1->STS1 & (1<<1)));
    (void)I2C1->STS2;
    I2C1->DR = reg;
    while (!(I2C1->STS1 & (1<<2)));
    I2C1->CTRL1 |= (1 << 8);
    while (!(I2C1->STS1 & (1<<0)));
    I2C1->DR = (addr << 1) | 1;
    while (!(I2C1->STS1 & (1<<1)));
    I2C1->CTRL1 &= ~(1 << 10);
    while (!(I2C1->STS1 & (1<<2)));
    uint8_t v = I2C1->DR;
    I2C1->CTRL1 |= (1 << 9);
    return v;
}

void uart_init(UART_t* u, uint32_t baud) {
    u->BAUD = APB2_HZ / (16 * baud);
    u->CTRL1 = (1<<3)|(1<<2)|(1<<13);
}

void uart_putc(UART_t* u, uint8_t b) {
    uint32_t to = 100000;
    while (!(u->STS & (1<<7))) { if(--to == 0) break; }
    u->DT = b;
}

bool uart_getc(UART_t* u, uint8_t *b) {
    if (u->STS & (1<<4)) {
        *b = (uint8_t)u->DT;
        return true;
    }
    return false;
}

void NVIC_SystemReset(void) {
    __disable_irq();
    __DSB();
    __ISB();
    SCB[0x0C >> 2] = 0x05FA0004U;
    while (1) {
        __NOP();
    }
}

void NVIC_SetPriority(uint8_t IRQn, uint32_t priority) {
    volatile uint8_t* ipr = (volatile uint8_t*)(0xE000E400 + IRQn);
    *ipr = (uint8_t)((priority << 4) & 0xFF);
}

