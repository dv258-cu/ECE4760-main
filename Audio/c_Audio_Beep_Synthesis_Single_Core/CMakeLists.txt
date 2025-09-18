# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.2.0)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.2.0)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pico CACHE STRING "Board type")

# cmake version
cmake_minimum_required(VERSION 3.13)

# include the sdk.cmake file
include(pico_sdk_import.cmake)

# give the project a name (anything you want)
project(Audio_Beep_Synth_Single_Core C CXX ASM)

# initialize the sdk
pico_sdk_init()

add_executable(Audio_Beep_Synth_Single_Core)

target_sources(Audio_Beep_Synth_Single_Core PRIVATE beep_beep.c)

# Add pico_multicore which is required for multicore functionality
target_link_libraries(Audio_Beep_Synth_Single_Core pico_stdlib pico_bootsel_via_double_reset hardware_spi hardware_sync pico_bootsel_via_double_reset)

# create map/bin/hex file etc.
pico_add_extra_outputs(Audio_Beep_Synth_Single_Core)