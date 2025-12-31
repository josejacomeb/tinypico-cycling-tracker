cmake_minimum_required(VERSION 3.20)
project(tinypico_ukf_test DESCRIPTION "Test UKF with GPS and IMU data"
        LANGUAGES CXX
)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
#set(CMAKE_CXX_EXTENSIONS OFF)

# Executable
add_executable(ukf_test
        src/main.cpp
        src/ukf_gps_imu.cpp
        src/ukf.cpp
        src/matrix.cpp
)

# Include directories
target_include_directories(ukf_test PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
)

# Compiler flags
if (MSVC)
    target_compile_options(ukf_test PRIVATE /W4)
else ()
    target_compile_options(ukf_test PRIVATE -Wall -Wextra -Wpedantic)
endif ()
