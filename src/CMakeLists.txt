add_subdirectory(gz)

set(BOXES_TEST_FILES
   boxes_dt_TEST.cc
   boxes_model_count_TEST.cc
)
message(STATUS ${BENCHMARK_EXE_SRC})

include(TestMacro.cmake)
set(TEST boxes)

gz_build_tests(${BOXES_TEST_FILES})