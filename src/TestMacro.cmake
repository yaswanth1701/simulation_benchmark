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


    add_executable(${BINARY_NAME} ${GTEST_SOURCE_file} ${BENCHMARK_EXE_SRC})

    target_include_directories(${BINARY_NAME} PUBLIC ${PROJECT_SOURCE_DIR}/include)

    target_link_libraries(${BINARY_NAME}
       gz-sim9 
       sdformat15 
       gz-math8 
       gtest 
       gtest_main
    )

    target_compile_definitions(${BINARY_NAME} PRIVATE TEST_NAME="${BINARY_NAME}")
    target_compile_definitions(${BINARY_NAME} PRIVATE PROJECT_SOURCE_DIR="${PROJECT_SOURCE_DIR}")

    add_test(${BINARY_NAME} ${CMAKE_CURRENT_BINARY_DIR}/${BINARY_NAME})

    set(_env_vars)
    set_tests_properties(${BINARY_NAME} PROPERTIES
      TIMEOUT 240
      ENVIRONMENT "${_env_vars}"
    )

    install(TARGETS ${BINARY_NAME}
      RUNTIME DESTINATION bin
    )
  endforeach()

  set(BENCHMARK_EXE_SRC "")
  set(TEST "")
endmacro()