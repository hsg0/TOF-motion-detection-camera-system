# Builds Pico 1 for the Pico W.
# The sources in this folder are the program that was flashed to the board.
#
#   cmake -S . -B build -DPICO_SDK_PATH=/path/to/pico-sdk -DPICO_BOARD=pico_w
#   cmake --build build
#
# Copy build/pico1Main.uf2 onto the Pico while it is in BOOTSEL mode.
# The boot drive is named RPI-RP2.

cmake_minimum_required(VERSION 3.13)

if (NOT PICO_SDK_PATH)
  if (DEFINED ENV{PICO_SDK_PATH})
    set(PICO_SDK_PATH $ENV{PICO_SDK_PATH})
  else()
    message(FATAL_ERROR "Set PICO_SDK_PATH to the Pico SDK")
  endif()
endif()

set(PICO_BOARD pico_w)
set(PICO_NO_FLASH 0)
set(PICO_COPY_TO_RAM 0)

include(${PICO_SDK_PATH}/pico_sdk_init.cmake)

project(pico1 C CXX ASM)
pico_sdk_init()

add_executable(pico1Main
  pico1Main.cpp
  spi-slave/pico1SpiLink.cpp
  vl53l0x/pico1Luna.cpp
)

pico_enable_stdio_uart(pico1Main 0)
pico_enable_stdio_usb(pico1Main 0)

target_include_directories(pico1Main PRIVATE
  ${CMAKE_CURRENT_LIST_DIR}
  ${CMAKE_CURRENT_LIST_DIR}/spi-slave
  ${CMAKE_CURRENT_LIST_DIR}/vl53l0x
)

target_link_libraries(pico1Main
  pico_stdlib
  pico_multicore
  hardware_gpio
  hardware_i2c
  hardware_uart
  hardware_watchdog
  pico_cyw43_arch_none
)

pico_add_extra_outputs(pico1Main)
