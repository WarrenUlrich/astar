cmake_minimum_required(VERSION 3.10)

project(astar VERSION 1.0)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED True)

add_library(
    astar INTERFACE
)

target_include_directories(
    astar INTERFACE include
)

enable_testing()
add_subdirectory(tests)