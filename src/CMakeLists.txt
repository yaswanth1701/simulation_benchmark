add_subdirectory(gz)

set(PROTO_FILES ${PROJECT_SOURCE_DIR}/proto/box_msg.proto)
include_directories(${Protobuf_INCLUDE_DIRS})
protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS ${PROTO_FILES})
         

set(BOXES_TEST_FILES
   boxes_dt_TEST.cc
   boxes_model_count_TEST.cc
)
message(STATUS ${BENCHMARK_EXE_SRC})

include(TestMacro.cmake)
set(TEST boxes)

gz_build_tests(${BOXES_TEST_FILES})