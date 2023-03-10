cmake_minimum_required(VERSION 3.5)
set(CMAKE_CXX_STANDARD 14)
add_compile_options(-Wall -Wextra -Wpedantic)

project(pyCandle)

set(CMAKE_BUILD_TYPE Release)

add_subdirectory(lib/pybind11)
pybind11_add_module(pyCandle bindings.cpp)

set(CANDLE_BUILD_STATIC TRUE)
set(CANDLE_OMIT_EXAMPLES TRUE)
add_subdirectory(src/candle)

if(NOT TARGET candle)
    add_library(candle STATIC)
endif()

target_link_libraries(pyCandle PUBLIC candle)
target_include_directories(pyCandle PUBLIC ${CMAKE_SOURCE_DIR}/include)
