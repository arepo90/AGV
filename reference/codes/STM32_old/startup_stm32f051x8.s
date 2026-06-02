    .syntax unified
    .cpu cortex-m0
    .thumb

    .global  Reset_Handler
    .global  _estack

    .word   _sidata
    .word   _sdata
    .word   _edata
    .word   _sbss
    .word   _ebss

/* Vector table */
    .section  .isr_vector,"a",%progbits
    .type  g_pfnVectors, %object

g_pfnVectors:
    .word  _estack
    .word  Reset_Handler
    .word  NMI_Handler
    .word  HardFault_Handler
    .word  0
    .word  0
    .word  0
    .word  0
    .word  0
    .word  0
    .word  0
    .word  SVC_Handler
    .word  0
    .word  0
    .word  PendSV_Handler
    .word  SysTick_Handler
    /* External IRQs 0..31 */
    .word  Default_Handler                  /*  0: WWDG                  */
    .word  Default_Handler                  /*  1: PVD / EXTI16          */
    .word  Default_Handler                  /*  2: RTC                   */
    .word  Default_Handler                  /*  3: FLASH                 */
    .word  Default_Handler                  /*  4: RCC                   */
    .word  Default_Handler                  /*  5: EXTI0_1               */
    .word  Default_Handler                  /*  6: EXTI2_3               */
    .word  EXTI4_15_IRQHandler              /*  7: EXTI4_15              */
    .word  Default_Handler                  /*  8: TSC                   */
    .word  DMA1_Channel1_IRQHandler         /*  9: DMA1_Channel1         */
    .word  DMA1_Channel2_3_IRQHandler       /* 10: DMA1_Channel2_3       */
    .word  DMA1_Channel4_5_IRQHandler       /* 11: DMA1_Channel4_5       */
    .word  Default_Handler                  /* 12: ADC1_COMP             */
    .word  Default_Handler                  /* 13: TIM1_BRK_UP_TRG_COM   */
    .word  Default_Handler                  /* 14: TIM1_CC               */
    .word  Default_Handler                  /* 15: TIM2                  */
    .word  Default_Handler                  /* 16: TIM3                  */
    .word  Default_Handler                  /* 17: TIM6_DAC              */
    .word  0                                /* 18: (reserved)            */
    .word  TIM14_IRQHandler                 /* 19: TIM14                 */
    .word  Default_Handler                  /* 20: TIM15                 */
    .word  Default_Handler                  /* 21: TIM16                 */
    .word  Default_Handler                  /* 22: TIM17                 */
    .word  I2C1_IRQHandler                  /* 23: I2C1                  */
    .word  Default_Handler                  /* 24: I2C2                  */
    .word  Default_Handler                  /* 25: SPI1                  */
    .word  Default_Handler                  /* 26: SPI2                  */
    .word  USART1_IRQHandler                /* 27: USART1                */
    .word  Default_Handler                  /* 28: USART2                */
    .word  0                                /* 29: (reserved)            */
    .word  Default_Handler                  /* 30: CEC / CAN             */
    .word  0                                /* 31: (reserved)            */

    .size  g_pfnVectors, .-g_pfnVectors

/* Reset handler: copy .data, zero .bss, call main */
    .section  .text.Reset_Handler
    .weak  Reset_Handler
    .type  Reset_Handler, %function
Reset_Handler:
    ldr   r0, =_estack
    mov   sp, r0

    /* copy .data from flash to RAM */
    ldr   r0, =_sdata
    ldr   r1, =_edata
    ldr   r2, =_sidata
    movs  r3, #0
copy_data:
    cmp   r0, r1
    bge   zero_bss
    ldr   r4, [r2, r3]
    str   r4, [r0, r3]
    adds  r3, r3, #4
    adds  r0, r0, #4
    b     copy_data

zero_bss:
    ldr   r0, =_sbss
    ldr   r1, =_ebss
    movs  r2, #0
zero_loop:
    cmp   r0, r1
    bge   call_main
    str   r2, [r0]
    adds  r0, r0, #4
    b     zero_loop

call_main:
    bl    main
hang:
    b     hang
    .size Reset_Handler, .-Reset_Handler

/* Default handler: infinite loop */
    .section  .text.Default_Handler,"ax",%progbits
    .type  Default_Handler, %function
Default_Handler:
    b  .
    .size  Default_Handler, .-Default_Handler

    .weak  NMI_Handler
    .thumb_set NMI_Handler,Default_Handler
    .weak  HardFault_Handler
    .thumb_set HardFault_Handler,Default_Handler
    .weak  SVC_Handler
    .thumb_set SVC_Handler,Default_Handler
    .weak  PendSV_Handler
    .thumb_set PendSV_Handler,Default_Handler
    .weak  SysTick_Handler
    .thumb_set SysTick_Handler,Default_Handler

    .weak  EXTI4_15_IRQHandler
    .thumb_set EXTI4_15_IRQHandler,Default_Handler
    .weak  DMA1_Channel1_IRQHandler
    .thumb_set DMA1_Channel1_IRQHandler,Default_Handler
    .weak  DMA1_Channel2_3_IRQHandler
    .thumb_set DMA1_Channel2_3_IRQHandler,Default_Handler
    .weak  DMA1_Channel4_5_IRQHandler
    .thumb_set DMA1_Channel4_5_IRQHandler,Default_Handler
    .weak  TIM14_IRQHandler
    .thumb_set TIM14_IRQHandler,Default_Handler
    .weak  I2C1_IRQHandler
    .thumb_set I2C1_IRQHandler,Default_Handler
    .weak  USART1_IRQHandler
    .thumb_set USART1_IRQHandler,Default_Handler
