macro (gz_build_tests)
  # Build all the tests
  foreach(GTEST_SOURCE_file ${ARGN})
    string(REGEX REPLACE ".cc" "" BINARY_NAME ${GTEST_SOURCE_file})
    set(BINARY_NAME ${TEST_TYPE}_${BINARY_NAME})

    if (EXISTS "${WORLDS_DIR}/${TEST}/${BINARY_NAME}")
          message(STATUS "${WORLDS_DIR_PATH}/${BINARY_NAME} exists!")
    else()
          execute_process(COMMAND mkdir  "${WORLDS_DIR}/${TEST}/${BINARY_NAME}")
          message(STATUS "${WORLDS_DIR_PATH}/${BINARY_NAME} created")
    endif()

    if (EXISTS "${TEST_RESULT_DIR}/${BINARY_NAME}")
          message(STATUS "${TEST_RESULT_DIR}/${BINARY_NAME}/MCAP exists!")
    else()
          execute_process(COMMAND mkdir ${TEST_RESULT_DIR}/${BINARY_NAME})
          execute_process(COMMAND mkdir ${TEST_RESULT_DIR}/${BINARY_NAME}/MCAP)

          message(STATUS "${TEST_RESULT_DIR}/${BINARY_NAME}/MCAP created")
    endif()


    add_executable(${BINARY_NAME} ${GTEST_SOURCE_file} ${BENCHMARK_EXE_SRC}
                                  ${PROTO_SRCS} ${PROTO_HDRS} ${PROTOBUF_DESCRIPTION_SRC})

 
    target_include_directories(${BINARY_NAME} PUBLIC ${PROJECT_SOURCE_DIR}/include)
    target_include_directories(${BINARY_NAME} PUBLIC ${CMAKE_BINARY_DIR}/src)
    target_include_directories(${BINARY_NAME} PUBLIC ${PROJECT_SOURCE_DIR}/mcap/cpp/mcap/include)
    target_include_directories(${BINARY_NAME} PUBLIC ${PROJECT_SOURCE_DIR}/mcap/cpp/examples)

    target_link_libraries(${BINARY_NAME}
       gz-sim9 
       sdformat15 
       gz-math8 
       gtest 
       gtest_main
       ${Protobuf_LIBRARIES}
       ${MCAP_DEPENDENCIES}    
       )

    target_compile_definitions(${BINARY_NAME} PRIVATE TEST_NAME="${BINARY_NAME}")
    target_compile_definitions(${BINARY_NAME} PRIVATE PROJECT_SOURCE_DIR="${PROJECT_SOURCE_DIR}")

    add_test(${BINARY_NAME} ${CMAKE_CURRENT_BINARY_DIR}/${BINARY_NAME})

    set_tests_properties(${BINARY_NAME} PROPERTIES
      TIMEOUT 1000
    )
    add_test(NAME csv_${BINARY_NAME}
      COMMAND python3 ${PROJECT_SOURCE_DIR}/tools/boxes/mcap_to_csv.py ${BINARY_NAME}
    )

    install(TARGETS ${BINARY_NAME}
      RUNTIME DESTINATION bin
    )
  endforeach()

  set(BENCHMARK_EXE_SRC "")
  set(TEST "")
endmacro()