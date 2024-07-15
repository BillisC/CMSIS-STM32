## GPIO Reader (WIP)

This project is about making a fast GPIO reader with a pulling rate of about 1us. The digital input pin is PA8.
Memory transfers are done using DMA2 Stream 5 (Channel 6).

*PA5 and PA8 pins shall be bridged for debugging*

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
- TIM1 : **180 MHz**
    - ARR : **179**
    - PSC : **0**

### Ports
- GPIOA :
    - PA5 : **OUT**
    - PA8 : **IN, PULLDOWN**