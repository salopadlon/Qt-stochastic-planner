cmake_minimum_required(VERSION 3.16)
project(qt_stochastic_planner LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(RRT_BUILD_TESTS "Build core unit tests" ON)
option(RRT_BUILD_CLI   "Build the CLI demo app" ON)

if(RRT_BUILD_TESTS)
    enable_testing()
endif()

add_subdirectory(core)

if(RRT_BUILD_CLI)
    add_subdirectory(apps/cli)
endif()