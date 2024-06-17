cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)
project(simulation_benchmark)
enable_testing()

find_package(gz-sim9 REQUIRED)
add_subdirectory(gtest_vendor)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(TEST_TYPE "BENCHMARK")
add_subdirectory(src)

