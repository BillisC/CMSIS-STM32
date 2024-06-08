#include <stdint.h>
#include "stm32f4xx.h"

#define LED_PIN 5

void systick_handler();
void delay_ms();

void main(void) {
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

    /* Configure flash controller for 3V3 and 100 MHz
     * system clock (3 wait states).
    */
    FLASH->ACR |= FLASH_ACR_LATENCY_3WS;

    /* Clear all PLL clock parameters */
    RCC->PLLCFGR &= ~(RCC_PLLCFGR_PLLM_Msk |
                      RCC_PLLCFGR_PLLN_Msk |
                      RCC_PLLCFGR_PLLP_Msk);

    /* Configure the PLL clock
     * 8 MHz / 4(M) = 2 MHz
     * 2 MHz * 200(N) = 400 MHz
     * 400 MHz / 4(P) = 100 MHz -> System Clock
     * 100 MHz / 2(Prescaler) = 50 MHz -> APB1
    */
    RCC->PLLCFGR |= ((4 << RCC_PLLCFGR_PLLM_Pos) |
                     (200 << RCC_PLLCFGR_PLLN_Pos) |
                     (1 << RCC_PLLCFGR_PLLP_Pos) |
                     (1 << RCC_PLLCFGR_PLLSRC_Pos));
    RCC->CFGR |= (0b100 << RCC_CFGR_PPRE1_Pos);

    /* Enable PLL clock and wait until it's ready */
    RCC->CR |= RCC_CR_PLLON_Msk;
    while (! (RCC->CR & RCC_CR_PLLRDY_Msk));

    /* Select PLL as the system clock source and
     * wait until it's ready.
    */
    RCC->CFGR |= (RCC_CFGR_SW_PLL << RCC_CFGR_SW_Pos);
    while (! (RCC->CFGR & RCC_CFGR_SWS_PLL));

    /* Inform CMSIS about the system clock */
    SystemCoreClockUpdate();

    /* Set interrupt rate to 1 KHz and enable interrupts */
    SysTick_Config(100000);
    __enable_irq();

    /* Enable AHB1->GPIOA */
    RCC->AHB1ENR |= (1 << RCC_AHB1ENR_GPIOAEN_Pos);
    /* Perform two dummy reads */
    dummy_read = RCC->AHB1ENR;
    dummy_read = RCC->AHB1ENR;

    /* Enable PA5 pin */
    GPIOA->MODER |= (1 << GPIO_MODER_MODER5_Pos);

    while (1) {
        /* Toggle PA5 pin output and wait for
         * x milliseconds.
        */
        GPIOA->ODR ^= (1 << LED_PIN);
        delay_ms(150);
    }
}

/* Redefine systick interrupt routine function since
 * it was set as weak.
 */
uint32_t ticks;
void systick_handler() {
    ticks++;
}

/* Simple delay function in milliseconds */
void delay_ms(uint32_t milliseconds) {
    uint32_t start = ticks;
    uint32_t end = start + milliseconds;

    if (end < start) {
        while (ticks > start);
    }

    while (ticks < end);
}
