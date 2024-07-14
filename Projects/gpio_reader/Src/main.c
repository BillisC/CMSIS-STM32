#include <stdint.h>
#include "stm32f446xx.h"
#include "stm32f4xx.h"

void systick_handler(void);
void delay_ms(uint32_t);
void clock_init(void);
void gpio_init(void);
void timer_init(void);
void dma_init(void);

volatile uint16_t readbuf;

void main(void) {
    /* Configure system clock */
    clock_init();

    /* Set interrupt rate to 1 KHz (/180 MHZ) and enable interrupts */
    SysTick_Config(180000);
    __enable_irq();

    /* Initialize peripherals */
    gpio_init();
    timer_init();
    dma_init();

    while (1) {
        delay_ms(200);
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

    /* PA8 pin is already in input mode */
    //GPIOA->MODER &= ~(0b00 << GPIO_MODER_MODER8_Pos);

    /* Set PA5 to output mode */
    GPIOA->MODER |= (0b01 << GPIO_MODER_MODER5_Pos);

    /* Set PA8 to pull down state  */
    GPIOA->PUPDR |= (0b10 << GPIO_PUPDR_PUPD8_Pos);

    /* Set PA5 to logic level 1 */
    GPIOA->ODR |= GPIO_ODR_OD5_Msk;
}

void timer_init(void) {
    /* Enable APB2->TIM1 */
    RCC->APB2ENR |= RCC_APB2ENR_TIM1EN_Msk;
    /* Perform two dummy reads */
    volatile uint32_t dummy_read;
    dummy_read = RCC->APB2ENR;
    dummy_read = RCC->APB2ENR;

    /* Configure the TIM1 clock
     * ARR + 1 = 180
     * PSC + 1 = 1
     * 180 * 1 / 180 MHz(fCLK) = 1 us
    */
    TIM1->ARR = 179;
    TIM1->PSC = 0;

    /* Configure for continuous counting and enable
     * DMA requests.
    */
    TIM1->EGR |= TIM_EGR_UG_Msk;
    TIM1->DIER |= TIM_DIER_UDE_Msk;

    /* Enable ARP and finally start the counter  */
    TIM1->CR1 |= (TIM_CR1_ARPE_Msk | TIM_CR1_CEN_Msk);
}

void dma_init(void) {
    /* Enable DMA2 clock */
    RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN_Msk;
    /* Perform two dummy reads */
    volatile uint32_t dummy_read;
    dummy_read = RCC->APB1ENR;
    dummy_read = RCC->APB1ENR;

    /* Configure DMA2 Stream 5 addresses */
    DMA2_Stream5->PAR |= (uint32_t)&GPIOA->IDR;
    DMA2_Stream5->M0AR |= (uint32_t)&readbuf;

    /* Configure the amount of data to transfer with the
     * DMA2 Stream 5.
    */
    DMA2_Stream5->NDTR |= (1 << DMA_SxNDT_Pos);

    /* Configure and enable DMA2 Stream 5:
     * Circular mode
     * Channel 6
     * Peripheral->Memory
     * Half-Word data size
    */
    DMA2_Stream5->CR |= ((DMA_SxCR_CIRC_Msk) |
                         (0b110 << DMA_SxCR_CHSEL_Pos) |
                         (0b01 << DMA_SxCR_MSIZE_Pos) |
                         (0b01 << DMA_SxCR_PSIZE_Pos) |
                         (DMA_SxCR_EN_Msk));
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
