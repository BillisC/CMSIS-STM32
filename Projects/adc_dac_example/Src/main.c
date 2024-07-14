#include <stdint.h>
#include "stm32f446xx.h"
#include "stm32f4xx.h"

#define LED_PIN 5

void systick_handler(void);
void delay_ms(uint32_t);
void clock_init(void);
void gpio_init(void);
void dma_init(void);
void adc_init(void);
void dac_init(void);

volatile uint32_t adc_buffer;
volatile uint32_t dac_buffer = 0;

void main(void) {
    /* Configure system clock */
    clock_init();

    /* Set interrupt rate to 1 KHz (/180 MHZ) and enable interrupts */
    SysTick_Config(180000);
    __enable_irq();

    /* Initialize peripherals */
    gpio_init();
    dma_init();
    adc_init();
    dac_init();

    /* Start ADC conversion */
    ADC1->CR2 |= ADC_CR2_SWSTART_Msk;

    while (1) {
        /* Toggle PA5 pin output and wait for
         * "X" milliseconds.
        */
        //GPIOA->ODR ^= (1 << LED_PIN);
        dac_buffer += 100;
        if (dac_buffer >= 4096)
            dac_buffer = 0;

        DAC->DHR12R2 = dac_buffer;
        delay_ms(200);
    }
}

void clock_init(void) {
    /* Enable 8 MHz HSE oscillator (Source: STLINK) and
     * wait until it's ready.
    */
    RCC->CR |= RCC_CR_HSEBYP_Msk | RCC_CR_HSEON_Msk;
    while (! (RCC->CR & RCC_CR_HSERDY_Msk));

    /* Enable power controller and DAC and change voltage
     * regulator scaling to 1.
    */
    RCC->APB1ENR |= RCC_APB1ENR_PWREN_Msk | RCC_APB1ENR_DACEN_Msk;
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
     * 180 MHz / 2(Prescaler) = 90 MHz -> APB2
    */
    RCC->PLLCFGR |= ((4 << RCC_PLLCFGR_PLLM_Pos) |
                     (180 << RCC_PLLCFGR_PLLN_Pos) |
                     // (0 << RCC_PLLCFGR_PLLP_Pos) |
                     (1 << RCC_PLLCFGR_PLLSRC_Pos));
    RCC->CFGR |= ((0b101 << RCC_CFGR_PPRE1_Pos) |
                  (0b100 << RCC_CFGR_PPRE2_Pos));

    /* Enable PLL clock and wait until it's ready */
    RCC->CR |= RCC_CR_PLLON_Msk;
    while (! (RCC->CR & RCC_CR_PLLRDY_Msk));

    /* Enable overdrive mode and wait until it's
     * ready.
    */
    PWR->CR |= (PWR_CR_ODEN_Msk);
    while (! (PWR->CSR & PWR_CSR_ODRDY));

    /* Switch internal voltage regulator to
     * overdrive mode and wait until it's ready.
    */
    PWR->CR |= (PWR_CR_ODSWEN_Msk);
    while (! (PWR->CSR & PWR_CSR_ODSWRDY));

    /* Select PLL as the system clock source and
     * wait until it's ready.
    */
    RCC->CFGR |= (RCC_CFGR_SW_PLL << RCC_CFGR_SW_Pos);
    while (! (RCC->CFGR & RCC_CFGR_SWS_PLL));

    /* Inform CMSIS about the system clock */
    SystemCoreClockUpdate();
}

void gpio_init(void) {
    /* Enable AHB1->GPIOA */
    RCC->AHB1ENR |= (RCC_AHB1ENR_GPIOAEN_Msk);
    /* Perform two dummy reads */
    volatile uint32_t dummy_read;
    dummy_read = RCC->AHB1ENR;
    dummy_read = RCC->AHB1ENR;

    /* Enable PA5 pin and PA0 for analog in */
    GPIOA->MODER |= ((0b11 << GPIO_MODER_MODER5_Pos) |
                     (0b11 << GPIO_MODER_MODER0_Pos));
}

void dma_init(void) {
    /* Enable DMA2 clock */
    RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN_Msk;
    /* Perform two dummy reads */
    volatile uint32_t dummy_read;
    dummy_read = RCC->APB1ENR;
    dummy_read = RCC->APB1ENR;

    /* Configure DMA2 Stream 0 addresses */
    DMA2_Stream0->PAR |= (uint32_t)&ADC1->DR;
    DMA2_Stream0->M0AR |= (uint32_t)&adc_buffer;

    /* Configure the amount of data to transfer with the
     * DMA2 Stream 0.
    */
    DMA2_Stream0->NDTR |= (1 << DMA_SxNDT_Pos);

    /* The DMA channel for Stream 0 is zero by
     * default so no changes needed at all for ADC1.
    */

    /* Configure and enable DMA2 Stream 0:
     * Circular mode
     * High priority
     * Peripheral->Memory
     * Half-Word data size
    */
    DMA2_Stream0->CR |= ((DMA_SxCR_CIRC_Msk) |
                         (0b01 << DMA_SxCR_MSIZE_Pos) |
                         (0b01 << DMA_SxCR_PSIZE_Pos) |
                         (DMA_SxCR_EN_Msk));
}

void adc_init(void) {
    /* Enable ADC1 clock */
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN_Msk;
    /* Perform two dummy reads */
    volatile uint32_t dummy_read;
    dummy_read = RCC->APB2ENR;
    dummy_read = RCC->APB2ENR;

    /* Configure the ADC Prescaler and wake temp sensor
     * 90 MHz(PPRE2) / 4 = 22.5 MHz -> ADC Clock
    */
    ADC->CCR |= (0b01 << ADC_CCR_ADCPRE_Pos);

    /* Set ADC1 resolution to 12 bits (>=15 cycles/conv)
    */
    ADC1->CR1 &= ~(0b11 << ADC_CR1_RES_Pos);

    /* Set sample time to 56 cycles (+ 12 = 68 cycles) */
    ADC1->SMPR2 |= (0b011 << ADC_SMPR2_SMP0_Pos);

    /* The amount of ADC1 conversions is set to 1 by default
     * and the order automatically starts with channel 0.
    */

    /* Configure the ADC1 for continuous mode with
     * DMA and enable it.
    */
    ADC1->CR2 |= ((ADC_CR2_CONT_Msk) |
                  (ADC_CR2_DDS_Msk) |
                  (ADC_CR2_DMA_Msk) |
                  (ADC_CR2_ADON_Msk));
}

void dac_init(void) {
    /* Enable DAC Channel 2 */
    DAC->CR |= DAC_CR_EN2_Msk;
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
