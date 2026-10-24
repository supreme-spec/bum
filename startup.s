.syntax unified
.cpu cortex-m4
.fpu fpv4-sp-d16
.thumb

.global g_pfnVectors
.g_pfnVectors:
.word _estack
.word Reset_Handler
.word NMI_Handler
.word HardFault_Handler
.word MemManage_Handler
.word BusFault_Handler
.word UsageFault_Handler
.word 0, 0, 0, 0
.word SVC_Handler
.word DebugMon_Handler
.word 0
.word PendSV_Handler
.word SysTick_Handler
.word DMA1_Ch1_IRQHandler
.word DMA1_Ch2_IRQHandler
.word DMA1_Ch3_IRQHandler
.word DMA1_Ch4_IRQHandler
.word DMA1_Ch5_IRQHandler
.word DMA1_Ch6_IRQhandler
.word DMA1_Ch7_IRQHandler
.word ADC_IRQHandler
.word 0, 0
.word EXTI0_IRQHandler
.word EXTI1_IRQHandler
.word EXTI2_IRQHandler
.word EXTI3_IRQHandler
.word EXTI4_IRQHandler
.word DMA1_Ch8_IRQHandler
.word DMA1_Ch9_IRQHandler
.word DMA1_Ch10_IRQHandler
.word DMA1_Ch11_IRQHandler
.word 0, 0
.word EXTI9_5_IRQHandler
.word TIM1_BRK_IRQHandler
.word TIM1_UP_IRQHandler
.word TIM1_TRG_COM_IRQHandler
.word TIM1_CC_IRQHandler
.word TIM2_IRQHandler
.word TIM3_IRQHandler
.word TIM4_IRQHandler
.word I2C1_EV_IRQHandler
.word I2C1_ER_IRQHandler
.word I2C2_EV_IRQHandler
.word I2C2_ER_IRQHandler
.word SPI1_IRQHandler
.word SPI2_IRQHandler
.word USART1_IRQHandler
.word USART2_IRQHandler
.word USART3_IRQHandler
.word EXTI15_10_IRQHandler
.word RTC_Alarm_IRQHandler
.word 0
.word TIM8_BRK_IRQHandler
.word TIM8_UP_IRQHandler
.word TIM8_TRG_COM_IRQHandler
.word TIM8_CC_IRQHandler
.word DMA1_Ch12_IRQHandler
.word 0, 0
.word SDIO1_IRQHandler
.word TIM5_IRQHandler
.word SPI3_IRQHandler
.word 0, 0
.word TIM6_DAC_IRQHandler
.word TIM7_IRQHandler
.word 0, 0, 0, 0, 0, 0, 0, 0
.word DMA1_Ch13_IRQHandler
.word DMA1_Ch14_IRQHandler
.word 0, 0, 0, 0, 0
.word USART6_IRQHandler

.section .text.Reset_Handler
.weak Reset_Handler
.type Reset_Handler, %function
Reset_Handler:
    ldr sp, =_estack
    bl SystemInit

    /* Copy .data from flash to RAM */
    ldr r2, =_sidata
    ldr r3, =_sdata
    ldr r4, =_edata
    cmp r3, r4
    beq .Ldata_copy_done
.Ldata_copy_loop:
    ldr r5, [r2], #4
    str r5, [r3], #4
    cmp r3, r4
    bne .Ldata_copy_loop
.Ldata_copy_done:

    /* Zero .bss */
    ldr r3, =_sbss
    ldr r4, =_ebss
    mov r5, #0
    cmp r3, r4
    beq .Lbss_done
.Lbss_loop:
    str r5, [r3], #4
    cmp r3, r4
    bne .Lbss_loop
.Lbss_done:

    bl main
    b .

.weak Default_Handler
.type Default_Handler, %function
Default_Handler: b .

.macro IRQ h
.weak \h
.set \h, Default_Handler
.endm

IRQ NMI_Handler
IRQ HardFault_Handler
IRQ MemManage_Handler
IRQ BusFault_Handler
IRQ UsageFault_Handler
IRQ SVC_Handler
IRQ DebugMon_Handler
IRQ PendSV_Handler
IRQ SysTick_Handler
IRQ DMA1_Ch1_IRQHandler
IRQ DMA1_Ch2_IRQHandler
IRQ DMA1_Ch3_IRQHandler
IRQ DMA1_Ch4_IRQHandler
IRQ DMA1_Ch5_IRQHandler
IRQ DMA1_Ch6_IRQhandler
IRQ DMA1_Ch7_IRQHandler
IRQ ADC_IRQHandler
IRQ EXTI0_IRQHandler
IRQ EXTI1_IRQHandler
IRQ EXTI2_IRQHandler
IRQ EXTI3_IRQHandler
IRQ EXTI4_IRQHandler
IRQ DMA1_Ch8_IRQHandler
IRQ DMA1_Ch9_IRQHandler
IRQ DMA1_Ch10_IRQHandler
IRQ DMA1_Ch11_IRQHandler
IRQ EXTI9_5_IRQHandler
IRQ TIM1_BRK_IRQHandler
IRQ TIM1_UP_IRQHandler
IRQ TIM1_TRG_COM_IRQHandler
IRQ TIM1_CC_IRQHandler
IRQ TIM2_IRQHandler
IRQ TIM3_IRQHandler
IRQ TIM4_IRQHandler
IRQ I2C1_EV_IRQHandler
IRQ I2C1_ER_IRQHandler
IRQ I2C2_EV_IRQHandler
IRQ I2C2_ER_IRQHandler
IRQ SPI1_IRQHandler
IRQ SPI2_IRQHandler
IRQ USART1_IRQHandler
IRQ USART2_IRQHandler
IRQ USART3_IRQHandler
IRQ EXTI15_10_IRQHandler
IRQ RTC_Alarm_IRQHandler
IRQ TIM8_BRK_IRQHandler
IRQ TIM8_UP_IRQHandler
IRQ TIM8_TRG_COM_IRQHandler
IRQ TIM8_CC_IRQHandler
IRQ DMA1_Ch12_IRQHandler
IRQ SDIO1_IRQHandler
IRQ TIM5_IRQHandler
IRQ SPI3_IRQHandler
IRQ TIM6_DAC_IRQHandler
IRQ TIM7_IRQHandler
IRQ DMA1_Ch13_IRQHandler
IRQ DMA1_Ch14_IRQHandler
IRQ USART6_IRQHandler
