cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(Firmware_CEREST LANGUAGES C CXX ASM)
message("Build type: " ${CMAKE_BUILD_TYPE})
     
# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
    Core/Src/puente.cpp
    Core/Src/Sensores/mpu6500.cpp
    Core/Src/Proceso/sd.cpp
    Core/Src/Sensores/qmc5883p.cpp
    Core/Src/Proceso/planificador.cpp
    Core/Src/Telemetria/telemetria_usb.cpp
    Core/Src/Control/mahony.cpp
    Core/Src/Control/ekf_Posicion.cpp
    Core/Src/Proceso/uart_dispatch.cpp
    Core/Src/Radio/crsf.cpp
    Core/Src/Radio/protocoloRadio.cpp
    Core/Src/Telemetria/telemetriaMAVlink.cpp
    Core/Src/Sensores/bmp180.cpp
    Core/Src/Control/pid.cpp
    Core/Src/Actuadores/motores.cpp
    Core/Src/Sensores/gnss.cpp
    Core/Src/Sensores/mtf02.cpp
    Core/Src/Actuadores/servo.cpp
    Core/Src/Sensores/bateria.cpp
    Core/Src/Sensores/buzzer.cpp
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
    Core/Inc
    Core/Src
    Core/Src/Sensores
    Core/Src/Proceso
    Core/Src/Telemetria
    Core/Src/Control
    Core/Src/Radio
    Core/Inc/Mavlink
    Core/Src/Actuadores
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

target_link_options(${PROJECT_NAME} PRIVATE
    -u _printf_float
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)
