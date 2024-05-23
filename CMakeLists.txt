cmake_minimum_required(VERSION 3.12)

include(pico_sdk_import.cmake)

project(rp2040_ov2640 C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# generate compile_commands.json for clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

pico_sdk_init()

add_executable(firmware
	ov2640.c
	main.c
)

pico_generate_pio_header(firmware ${CMAKE_CURRENT_LIST_DIR}/picampinos.pio)

target_link_libraries(firmware
	pico_stdlib
	hardware_dma
	hardware_i2c
	hardware_pio
	hardware_pwm
)

pico_enable_stdio_usb(firmware 1)
pico_enable_stdio_uart(firmware 0)

pico_add_extra_outputs(firmware)
