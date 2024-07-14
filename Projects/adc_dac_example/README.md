## ADC & DAC Example

This project is about configuring, initializing, and using the ADC and DAC hardware components of STM32. Memory transfers are done using DMA2 Stream 0 (Channel 0).

*PA0 and PA5 pins have to be bridged!*

### Timings
- Clock Source : **HSE (8MHz)**
- System Clock : **180 MHz**
    - PLLM : **4**
    - PLLN : **180**
    - PLLP : **2**
- APB1 : **45 MHz**
    - Prescaler : **4**
- APB2 : **90 MHz**
    - Prescaler : **2**

