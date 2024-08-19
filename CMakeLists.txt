cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)
project(simulation_benchmark)
enable_testing()

find_package(gz-sim9 REQUIRED)
add_subdirectory(gtest_vendor)

find_package(PkgConfig REQUIRED)
pkg_check_modules(zstd REQUIRED IMPORTED_TARGET libzstd)
pkg_check_modules(lz4 REQUIRED IMPORTED_TARGET liblz4)

find_package(protobuf CONFIG)
if(NOT protobuf_FOUND)
 find_package(Protobuf REQUIRED)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(MCAP_DEPENDENCIES PkgConfig::lz4 PkgConfig::zstd)

set(TEST_RESULT_DIR "${PROJECT_SOURCE_DIR}/test_results")
set(WORLDS_DIR ${PROJECT_SOURCE_DIR}/worlds)
set(TEST_TYPE "BENCHMARK")

set(PROTOBUF_DESCRIPTION_DIR ${PROJECT_SOURCE_DIR}/mcap/cpp/examples/protobuf/)
set(PROTOBUF_DESCRIPTION_SRC 
    ${PROTOBUF_DESCRIPTION_DIR}/BuildFileDescriptorSet.cpp
)
message(${PROTOBUF_DESCRIPTION_SRC})

add_subdirectory(src)

