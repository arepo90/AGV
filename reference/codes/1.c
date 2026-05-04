// Example code for other unrelated task

#include "stm32f0xx.h"
#include <stdint.h>

uint8_t motor = 0;

void delay(uint32_t time) {
    while (time--) {
    	__asm("nop");
    }
}

int main(void) {
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
    RCC->AHBENR |= RCC_AHBENR_GPIOCEN;

    GPIOA->PUPDR |= GPIO_PUPDR_PUPDR0_1;
    GPIOC->MODER |= GPIO_MODER_MODER8_0;
    GPIOC->MODER |= GPIO_MODER_MODER9_0;
    GPIOC->OTYPER |= GPIO_OTYPER_OT_9;

    while (1) {
        if (GPIOA->IDR & GPIO_IDR_0) {
            delay(40000);
            if (GPIOA->IDR & GPIO_IDR_0) {
                motor ^= 1;
                if (motor) {
                    GPIOC->BSRR = GPIO_BSRR_BS_8;
                    GPIOC->BSRR = GPIO_BSRR_BS_9;
                }
                else {
                    GPIOC->BSRR = GPIO_BSRR_BR_8;
                    GPIOC->BSRR = GPIO_BSRR_BR_9;
                }
                while (GPIOA->IDR & GPIO_IDR_0);
                delay(40000);
            }
        }
    }
}