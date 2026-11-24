cmake_minimum_required(VERSION 3.21)
project(calibrate LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_compile_definitions(UNICODE _UNICODE NOMINMAX)

if(MSVC)
    add_compile_options(/W4 /permissive- /utf-8)
endif()

add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../plant ${CMAKE_CURRENT_BINARY_DIR}/plant)

add_executable(calibrate
    main.cpp
    Runtime.cpp
    ExperimentUtils.cpp
    experiments/ThrottleStep.cpp
    experiments/HoverTrim.cpp
    experiments/AxisPulse.cpp
    experiments/ValidateForecast.cpp
    experiments/Stubs.cpp
)
target_include_directories(calibrate PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/../vision
)
target_link_libraries(calibrate PRIVATE plant winmm)
