#include <stdint.h>
#include "stm32f4xx.h"

#define LED_PIN 5

void systick_handler(void);
void delay_ms(uint32_t);
void clock_init(void);
void dma_init(void);

void main(void) {
    /* Configure system clock */
    clock_init();

    /* Set interrupt rate to 1 KHz (/180 MHZ) and enable interrupts */
    SysTick_Config(180000);
    __enable_irq();

    /* Enable AHB1->GPIOA */
    RCC->AHB1ENR |= (1 << RCC_AHB1ENR_GPIOAEN_Pos);
    /* Perform two dummy reads */
    volatile uint32_t dummy_read;
    dummy_read = RCC->AHB1ENR;
    dummy_read = RCC->AHB1ENR;

    /* Enable PA5 pin */
    GPIOA->MODER |= (1 << GPIO_MODER_MODER5_Pos);

    while (1) {
        /* Toggle PA5 pin output and wait for
         * "X" milliseconds.
        */
        GPIOA->ODR ^= (1 << LED_PIN);
        delay_ms(1000);
    }
}

void clock_init(void) {
    /* Enable 8 MHz HSE oscillator (Source: STLINK) and
     * wait until it's ready.
    */
    RCC->CR |= RCC_CR_HSEBYP_Msk | RCC_CR_HSEON_Msk;
    while (! (RCC->CR & RCC_CR_HSERDY_Msk));

    /* Enable power controller and change voltage
     * regulator scaling to 1.
    */
    RCC->APB1ENR |= RCC_APB1ENR_PWREN_Msk;
    /* Perform two dummy reads */
    volatile uint32_t dummy_read;
    dummy_read = RCC->APB1ENR;
    dummy_read = RCC->APB1ENR;
    PWR->CR |= (0b11 << PWR_CR_VOS_Pos);

    /* Configure flash controller for 3V3 and 180 MHz
     * system clock (5 wait states).
    */
    FLASH->ACR |= FLASH_ACR_LATENCY_5WS;

    /* Clear all PLL clock parameters */
    RCC->PLLCFGR &= ~(RCC_PLLCFGR_PLLM_Msk |
                      RCC_PLLCFGR_PLLN_Msk |
                      RCC_PLLCFGR_PLLP_Msk);

    /* Configure the PLL clock
     * 8 MHz / 4(M) = 2 MHz
     * 2 MHz * 180(N) = 360 MHz
     * 360 MHz / 2(P) = 180 MHz -> System Clock
     * 180 MHz / 4(Prescaler) = 45 MHz -> APB1
    */
    RCC->PLLCFGR |= ((4 << RCC_PLLCFGR_PLLM_Pos) |
                     (180 << RCC_PLLCFGR_PLLN_Pos) |
                     (0 << RCC_PLLCFGR_PLLP_Pos) | /* Already by default */
                     (1 << RCC_PLLCFGR_PLLSRC_Pos));
    RCC->CFGR |= (0b101 << RCC_CFGR_PPRE1_Pos);

    /* Enable PLL clock and wait until it's ready */
    RCC->CR |= RCC_CR_PLLON_Msk;
    while (! (RCC->CR & RCC_CR_PLLRDY_Msk));

    /* Enable overdrive mode and wait until it's
     * ready.
    */
    PWR->CR |= (1 << PWR_CR_ODEN_Pos);
    while (! (PWR->CSR & PWR_CSR_ODRDY));

    /* Switch internal voltage regulator to
     * overdrive mode and wait until it's ready.
    */
    PWR->CR |= (1 << PWR_CR_ODSWEN_Pos);
    while (! (PWR->CSR & PWR_CSR_ODSWRDY));

    /* Select PLL as the system clock source and
     * wait until it's ready.
    */
    RCC->CFGR |= (RCC_CFGR_SW_PLL << RCC_CFGR_SW_Pos);
    while (! (RCC->CFGR & RCC_CFGR_SWS_PLL));

    /* Inform CMSIS about the system clock */
    SystemCoreClockUpdate();
}

void dma_init() {
    /* Configure the APB2 Prescaler
     * 180 MHz(SYSCLK) / 2 = 90 MHz -> APB2
    */
    RCC->CFGR |= (0b100 << RCC_CFGR_PPRE2_Pos);

    /* Configure the ADC Prescaler
     * 90 MHz(PPRE2) / 4 = 22.5 MHz -> ADC Clock
    */
    ADC->CCR |= (0b01 << ADC_CCR_ADCPRE_Pos);

    /* Enable ADC1 clock */
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN_Msk;
    /* Perform two dummy reads */
    volatile uint32_t dummy_read;
    dummy_read = RCC->APB1ENR;
    dummy_read = RCC->APB1ENR;

    /* Set ADC resolution to 12 bits (>=15 cycles/conv) */
    ADC->CR1 |= (0b00 << ADC_CR1_RES_Pos) // This has no effect

    /* Configure the ADC for continuous mode and
     * enable it.
    */
    ADC->CR2 |= ((1 << ADC_CR2_CONT_Pos) |
                 (1 << ADC_CR2_ADON_Pos));
}

/* Redefine systick interrupt routine function since
 * it was set as weak.
*/
uint32_t ticks;
void systick_handler(void) {
    ticks++;
}

/* Simple delay function in milliseconds */
void delay_ms(uint32_t milliseconds) {
    uint32_t start = ticks;
    uint32_t end = start + milliseconds;

    if (end < start)
        while (ticks > start);

    while (ticks < end);
}
