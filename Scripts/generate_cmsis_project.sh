# | ----- CMSIS 5 - STM32 Project -----| #
# |            by BillisCh             | #
# |------------------------------------| #
echo "Generating CMSIS 5 project"
echo "Target: STM32F4xx"

echo "Install prerequisites.."
sudo apt update && sudo apt install git -y

echo "Setting up project structure.."
mkdir -p project/
mkdir -p project/vendor/
touch project/main.c
touch project/startup.c
touch project/linker_script.ld

echo "Cloning CMSIS repo.."
cd project/vendor/
git clone https://github.com/ARM-software/CMSIS_5 CMSIS && cd CMSIS/

echo "Cloning STM32F4xx Device repo.."
rm -rf Device/*
mkdir -p Device/ST
git clone https://github.com/STMicroelectronics/cmsis_device_f4 Device/ST/STM32F4

echo "Removing uneeded stuff.."
rm -rf .git* Jenk* CMSIS_Re* Scalab* docker ARM.CMSIS*
rm -rf CMSIS/CoreVa* CMSIS/Core_A/ CMSIS/DAP/ CMSIS/DSP/ CMSIS/Doxy* CMSIS/Driver/ CMSIS/NN/ CMSIS/Pack/ CMSIS/RTOS/ CMSIS/RTOS2/ CMSIS/Uti*

echo "You may rename the project folder to something else"
echo "Finito."
