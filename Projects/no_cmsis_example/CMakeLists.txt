cmake_minimum_required(VERSION 3.20)
project(exec VERSION 1.0)

# GCC ARM compiler flags
set(CMAKE_C_COMPILER arm-none-eabi-gcc)
set(CMAKE_C_FLAGS "-g -mcpu=cortex-m4 -mthumb -nostdlib -DSTM32F446xx -Wl,-Map=${PROJECT_NAME}.map")

# Linker options
set(LINKER_SCRIPT "linker_script.ld")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -T ${LINKER_SCRIPT}")

# Needed to make clangd functional
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_executable(${PROJECT_NAME}.elf Src/main.c Src/startup_f446re.c)

target_include_directories(${PROJECT_NAME}.elf PUBLIC)

set_target_properties(${TARGET_NAME} PROPERTIES LINK_DEPENDS ${LINKER_SCRIPT})

# Flashing procedure
set(PROGRAMMER openocd)
set(PROGRAMMER_FLAGS -f interface/stlink.cfg -f target/stm32f4x.cfg)

add_custom_target(flash
    COMMAND ${PROGRAMMER} ${PROGRAMMER_FLAGS} -c "program ${CMAKE_BINARY_DIR}/${PROJECT_NAME}.elf verify reset exit"
    DEPENDS ${PROJECT_NAME}.elf
    COMMENT "Flashing the chip"
)

# Debugging procedure
add_custom_target(debug
    COMMAND ${PROGRAMMER} ${PROGRAMMER_FLAGS}
    DEPENDS ${PROJECT_NAME}.elf
    COMMENT "Debugging the chip"
)
