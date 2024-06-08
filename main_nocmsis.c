#include <stdint.h>

#define PERIPHERAL_BASE (0x40000000U)
#define AHB1_BASE (PERIPHERAL_BASE + 0x20000U)
#define GPIOA_BASE (AHB1_BASE + 0x0U)
#define RCC_BASE (AHB1_BASE + 0x3800U)

#define RCC_AHB1ENR_OFFSET (0x30U)
#define RCC_AHB1ENR ((volatile uint32_t*) (RCC_BASE + RCC_AHB1ENR_OFFSET))
#define RCC_AHB1ENR_GPIOAEN (0x00U) /* bit 0 */

#define GPIO_MODER_OFFSET (0x00U)
#define GPIOA_MODER ((volatile uint32_t*) (GPIOA_BASE + GPIO_MODER_OFFSET))
#define GPIO_MODER_MODER5 (10U)
#define GPIO_ODR_OFFSET (0x14U)
#define GPIOA_ODR ((volatile uint32_t*) (GPIOA_BASE + GPIO_ODR_OFFSET))

#define LED_PIN 5

void main(void) {
    *RCC_AHB1ENR |= (1 << RCC_AHB1ENR_GPIOAEN);

    /* Perform two required dummy reads */
    volatile uint32_t dummy_read;
    dummy_read = *(RCC_AHB1ENR);
    dummy_read = *(RCC_AHB1ENR);

    *GPIOA_MODER |= (1 << GPIO_MODER_MODER5);

    while (1) {
        *GPIOA_ODR ^= (1 << LED_PIN);

        /* Dumb wait function */
        for (uint32_t i = 0; i < 1000000; i++);
    }

}
