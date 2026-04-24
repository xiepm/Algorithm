cmake_minimum_required(VERSION 3.10)
project(CmpHansCobotAlgorithm LANGUAGES C CXX)

set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type")
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(COMMON_WARNING_FLAGS
    -s
    -Wall
    -Wextra
    -fvisibility=hidden
    -fvisibility-inlines-hidden
)

add_compile_options(${COMMON_WARNING_FLAGS})

set(PROJECT_EXTERNAL_DIR "${CMAKE_CURRENT_SOURCE_DIR}/external")
set(CODESYS_COMPONENTS_DIR "${PROJECT_EXTERNAL_DIR}/Components")
set(CODESYS_PLATFORM_DIR "${PROJECT_EXTERNAL_DIR}/Platforms/Linux")

find_path(EIGEN3_INCLUDE_DIR
    NAMES Eigen/Eigen
    HINTS
        "${PROJECT_EXTERNAL_DIR}/eigen3"
        "${PROJECT_EXTERNAL_DIR}/include/eigen3"
        "${CMAKE_CURRENT_SOURCE_DIR}/include/eigen3"
    PATHS
        /usr/include/eigen3
        /usr/local/include/eigen3
)

if(NOT EIGEN3_INCLUDE_DIR)
    message(FATAL_ERROR
        "Could not find Eigen headers (expected Eigen/Eigen). "
        "Install Eigen3 or place it under external/eigen3."
    )
endif()

foreach(required_dir IN ITEMS
    "${CODESYS_COMPONENTS_DIR}"
    "${CODESYS_PLATFORM_DIR}"
)
    if(NOT EXISTS "${required_dir}")
        message(FATAL_ERROR "Required dependency directory not found: ${required_dir}")
    endif()
endforeach()

include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${CMAKE_CURRENT_SOURCE_DIR}/src/dynamics
    ${CODESYS_COMPONENTS_DIR}
    ${CODESYS_PLATFORM_DIR}
    ${EIGEN3_INCLUDE_DIR}
)

aux_source_directory(src srcs)
aux_source_directory(src/dynamics srcs)

add_library(CmpHansCobotAlgorithm SHARED
    ${srcs}
    ElfinCollaborativeLib.c
    algorithmInterface.cpp
)

aux_source_directory(test TEST)
list(REMOVE_ITEM TEST "test/torqueCompensationTest.cpp")
add_executable(testDemo
    ${TEST}
    ${srcs}
)

add_executable(torqueCompensationTest
    test/torqueCompensationTest.cpp
    ${srcs}
)
