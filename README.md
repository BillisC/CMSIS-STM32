# CMSIS-STM32
Personal repository for STM32 related projects (with CMSIS).

Clone this repo using:
```
git clone --recursive https://github.com/BillisC/CMSIS-STM32.git
```
or if you already cloned it:
```
git submodule init
git submodule update
```

### Required tools for compilation and programming
An ARM compiler xD (~2.5 GB):
```
sudo apt install gcc-arm-none-eabi gdb-multiarch -y
```
OpenOCD is the recommended programmer (supports STLINK out-of-the-box):
```
sudo apt install openocd -y
```
And of course our beloved scripting language (not):
```
sudo apt install cmake -y
```

### Compile & flash
Configure every project with:
```
cmake .
```
Connect the STLINK programmer on your personal computer and run:
```
cmake --build . --target flash
```

### Debugging
Debugging is made easy:
```
cmake --build . --target debug
```
Then use another gdb instance and connect to port 3333:
```
gdb-multiarch -tui blink.elf
(gdb) target remote :3333
(gdb) br main
```

### Projects
- ADC & DAC implementation
- No CMSIS implementation
- A very fast GPIO reader

### Submodules
-  STM32F4xx CMSIS Device Files - https://github.com/STMicroelectronics/cmsis_device_f4
-  CMSIS 5 - https://github.com/ARM-software/CMSIS_5 (stripped down + pre-included)

<img src="https://img1.daumcdn.net/thumb/R800x0/?scode=mtistory2&fname=https%3A%2F%2Fblog.kakaocdn.net%2Fdn%2FbHD7Cc%2Fbtr0oAASgS8%2FlNeBfxpoVtgVsKVdrXyeLk%2Fimg.png" width="100px">
