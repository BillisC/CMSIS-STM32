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
sudo apt install gcc-arm-none-eabi gdb-arm-none-eabi -y
```
OpenOCD is the recommended programmer (supports STLINK out-of-the-box):
```
sudo apt install openocd -y
```
And of course our beloved scripting language (not):
```
sudo apt install make -y
```
### Compile & flash
Connect the STLINK programmer on your personal computer and run:
```
make flash
```
this should build the project, detect the board, and finally flash it.

### Submodules:
-  STM32F4xx CMSIS Device Files - https://github.com/STMicroelectronics/cmsis_device_f4
-  CMSIS 5 - https://github.com/ARM-software/CMSIS_5 (stripped down + pre-included)
