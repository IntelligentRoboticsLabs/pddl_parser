ament_add_gtest(utils_test utils_test.cpp)
target_link_libraries(utils_test ${PROJECT_NAME})

set(TEST_PDDL_DIR ${CMAKE_CURRENT_SOURCE_DIR}/pddl)

install(DIRECTORY
  ${TEST_PDDL_DIR}
  DESTINATION share/${PROJECT_NAME}
)
