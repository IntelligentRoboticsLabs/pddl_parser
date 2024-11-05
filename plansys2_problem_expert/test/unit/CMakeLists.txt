ament_add_gtest(utils_test utils_test.cpp TIMEOUT 1200)
target_link_libraries(utils_test ${PROJECT_NAME})

ament_add_gtest(problem_expert_test problem_expert_test.cpp TIMEOUT 1200)
target_link_libraries(problem_expert_test ${PROJECT_NAME})

ament_add_gtest(problem_expert_node_test problem_expert_node_test.cpp TIMEOUT 1200)
target_link_libraries(problem_expert_node_test ${PROJECT_NAME})
