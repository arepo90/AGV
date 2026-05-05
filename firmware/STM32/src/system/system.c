#include "system.h"
#include "config.h"
#include "stm32f0xx.h"

static volatile uint32_t s_ms = 0;
static reset_cause_t     s_reset_cause = RESET_CAUSE_UNKNOWN;

void SysTick_Handler(void) {
    s_ms++;
}

uint32_t system_now_ms(void) {
    /* 32-bit aligned read on Cortex-M0 is atomic — no critical section needed. */
    return s_ms;
}

reset_cause_t system_reset_cause(void) {
    return s_reset_cause;
}

static void capture_reset_cause(void) {
    uint32_t csr = RCC->CSR;
    if      (csr & RCC_CSR_LPWRRSTF) s_reset_cause = RESET_CAUSE_LOW_POWER;
    else if (csr & RCC_CSR_WWDGRSTF) s_reset_cause = RESET_CAUSE_WATCHDOG;
    else if (csr & RCC_CSR_IWDGRSTF) s_reset_cause = RESET_CAUSE_WATCHDOG;
    else if (csr & RCC_CSR_SFTRSTF)  s_reset_cause = RESET_CAUSE_SOFTWARE;
    else if (csr & RCC_CSR_PINRSTF)  s_reset_cause = RESET_CAUSE_PIN;
    else if (csr & RCC_CSR_PORRSTF)  s_reset_cause = RESET_CAUSE_POWER_ON;
    RCC->CSR |= RCC_CSR_RMVF;   /* clear flags so next reset is captured cleanly */
}

static void clock_init_48mhz(void) {
    /* HSI is 8 MHz, on by default. Configure PLL: HSI/2 × 12 = 48 MHz. */
    /* 1 wait state required for SYSCLK > 24 MHz. Enable prefetch for performance. */
    FLASH->ACR = FLASH_ACR_PRFTBE | FLASH_ACR_LATENCY;

    /* Make sure PLL is off before reconfiguring. */
    RCC->CR &= ~RCC_CR_PLLON;
    while (RCC->CR & RCC_CR_PLLRDY) { }

    /* PLLSRC = HSI/2 (CFGR.PLLSRC = 0), PLLMUL = ×12 → 4 × 12 = 48 MHz. */
    RCC->CFGR = (RCC->CFGR & ~(RCC_CFGR_PLLSRC | RCC_CFGR_PLLMUL))
              | RCC_CFGR_PLLSRC_HSI_DIV2
              | RCC_CFGR_PLLMUL12;

    /* APB and AHB prescalers = /1 (PCLK = HCLK = SYSCLK = 48 MHz). */
    RCC->CFGR &= ~(RCC_CFGR_HPRE | RCC_CFGR_PPRE);

    RCC->CR |= RCC_CR_PLLON;
    while (!(RCC->CR & RCC_CR_PLLRDY)) { }

    /* Switch SYSCLK to PLL. */
    RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_SW) | RCC_CFGR_SW_PLL;
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL) { }
}

static void systick_init(void) {
    /* SysTick from SYSCLK; reload = SYSCLK / SYSTICK_HZ. */
    SysTick_Config(SYSCLK_HZ / SYSTICK_HZ);
}

void system_init(void) {
    capture_reset_cause();
    clock_init_48mhz();
    systick_init();
}

#if ENABLE_IWDG
/* IWDG runs from LSI (~40 kHz). Prescaler /32 → 1.25 kHz tick → 0.8 ms/count.
 * RLR is 12-bit (max 4095). For IWDG_TIMEOUT_MS, reload = ms × 1.25 (rounded). */
void system_iwdg_init(void) {
    IWDG->KR = 0x5555u;  /* enable register access */
    IWDG->PR = 3u;       /* prescaler /32 */

    uint32_t reload = (IWDG_TIMEOUT_MS * 1250u) / 1000u;
    if (reload == 0)    reload = 1;
    if (reload > 4095u) reload = 4095u;
    IWDG->RLR = reload;

    while (IWDG->SR) { }     /* wait for PR/RLR update to complete */
    IWDG->KR = 0xAAAAu;      /* reload counter */
    IWDG->KR = 0xCCCCu;      /* start watchdog */
}

void system_iwdg_pet(void) {
    IWDG->KR = 0xAAAAu;
}
#else
void system_iwdg_init(void) { }
void system_iwdg_pet(void)  { }
#endif

void system_soft_reset(void) {
    __DSB();
    SCB->AIRCR = (0x5FAu << SCB_AIRCR_VECTKEY_Pos) | SCB_AIRCR_SYSRESETREQ_Msk;
    for (;;) { }
}
