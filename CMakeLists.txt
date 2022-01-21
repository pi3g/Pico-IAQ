cmake_minimum_required(VERSION 3.13)

include(pico_sdk_import.cmake)

project(IAQ_Pico C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

add_executable(${PROJECT_NAME}
    IAQ_Pico.c
)

add_library(algobsec STATIC IMPORTED)
set_property(TARGET algobsec PROPERTY IMPORTED_LOCATION libalgobsec.a)

add_library(integration bsec_integration.h bsec_integration.c)
add_library(bme680 bme680.h bme680.c)
add_library(serialized_conf bsec_serialized_configurations_iaq.h bsec_serialized_configurations_iaq.c)

pico_enable_stdio_usb(${PROJECT_NAME} 1)
pico_enable_stdio_uart(${PROJECT_NAME} 1)

pico_add_extra_outputs(${PROJECT_NAME})

# Link libraries libname.a or libname.so -> name
target_link_libraries(${PROJECT_NAME}
	pico_stdlib
	pico_multicore
	hardware_i2c
	hardware_gpio
	hardware_adc
	algobsec
	integration
	bme680
	serialized_conf
)