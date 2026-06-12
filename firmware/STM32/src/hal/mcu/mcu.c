#include "mcu.h"
#include "config.h"
#include "stm32f0xx.h"

static volatile uint32_t s_ms = 0;
static reset_cause_t     s_reset_cause = RESET_CAUSE_UNKNOWN;

void SysTick_Handler(void) {
    s_ms++;
}

uint32_t mcu_now_ms(void) {
    /* 32-bit aligned read is atomic on Cortex-M0 — no critical section needed. */
    return s_ms;
}

reset_cause_t mcu_reset_cause(void) {
    return s_reset_cause;
}

void mcu_delay_ms(uint32_t ms) {
    uint32_t start = s_ms;
    while ((s_ms - start) < ms) { }
}

static void capture_reset_cause(void) {
    /* POR/PDR (power-on or brown-out) also asserts NRST, setting PINRSTF
     * alongside PORRSTF — so POR must be checked before PIN or every power
     * dip reads as an external pin reset. PIN alone = a genuine NRST pull. */
    uint32_t csr = RCC->CSR;
    if      (csr & RCC_CSR_LPWRRSTF) s_reset_cause = RESET_CAUSE_LOW_POWER;
    else if (csr & RCC_CSR_WWDGRSTF) s_reset_cause = RESET_CAUSE_WATCHDOG;
    else if (csr & RCC_CSR_IWDGRSTF) s_reset_cause = RESET_CAUSE_WATCHDOG;
    else if (csr & RCC_CSR_SFTRSTF)  s_reset_cause = RESET_CAUSE_SOFTWARE;
    else if (csr & RCC_CSR_PORRSTF)  s_reset_cause = RESET_CAUSE_POWER_ON;
    else if (csr & RCC_CSR_PINRSTF)  s_reset_cause = RESET_CAUSE_PIN;
    RCC->CSR |= RCC_CSR_RMVF;   /* clear flags so the next reset is captured cleanly */
}

static void clock_init_48mhz(void) {
    /* HSI is 8 MHz, on by default. PLL = (HSI/2) × 12 = 48 MHz.
     * 1 flash wait state required above 24 MHz; enable the prefetch buffer. */
    FLASH->ACR = FLASH_ACR_PRFTBE | FLASH_ACR_LATENCY;

    RCC->CR &= ~RCC_CR_PLLON;
    while (RCC->CR & RCC_CR_PLLRDY) { }

    RCC->CFGR = (RCC->CFGR & ~(RCC_CFGR_PLLSRC | RCC_CFGR_PLLMUL))
              | RCC_CFGR_PLLSRC_HSI_DIV2
              | RCC_CFGR_PLLMUL12;
    RCC->CFGR &= ~(RCC_CFGR_HPRE | RCC_CFGR_PPRE);   /* AHB = APB = SYSCLK */

    RCC->CR |= RCC_CR_PLLON;
    while (!(RCC->CR & RCC_CR_PLLRDY)) { }

    RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_SW) | RCC_CFGR_SW_PLL;
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL) { }
}

void mcu_init(void) {
    capture_reset_cause();
    clock_init_48mhz();
    SysTick_Config(SYSCLK_HZ / SYSTICK_HZ);
}

#if ENABLE_IWDG
/* IWDG runs from LSI (~40 kHz). Prescaler /32 → ~1.25 kHz tick (0.8 ms/count).
 * RLR is 12-bit (max 4095): reload = ms × 1.25. */
void mcu_iwdg_init(void) {
    IWDG->KR = 0xCCCCu;   /* start watchdog (also enables LSI) */
    IWDG->KR = 0x5555u;   /* unlock PR/RLR */
    IWDG->PR = 3u;        /* /32 prescaler */

    uint32_t reload = (IWDG_TIMEOUT_MS * 1250u) / 1000u;
    if (reload == 0)     reload = 1;
    if (reload > 4095u)  reload = 4095u;
    IWDG->RLR = reload;

    while (IWDG->SR) { }  /* wait for PR/RLR sync into the LSI domain */
    IWDG->KR = 0xAAAAu;   /* reload counter with new RLR */
}

void mcu_iwdg_pet(void) {
    IWDG->KR = 0xAAAAu;
}
#else
void mcu_iwdg_init(void) { }
void mcu_iwdg_pet(void)  { }
#endif

void mcu_soft_reset(void) {
    __DSB();
    SCB->AIRCR = (0x5FAu << SCB_AIRCR_VECTKEY_Pos) | SCB_AIRCR_SYSRESETREQ_Msk;
    for (;;) { }
}
