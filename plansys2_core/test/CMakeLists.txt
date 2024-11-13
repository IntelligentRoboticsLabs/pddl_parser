ament_add_gtest(utils_test utils_test.cpp)
target_link_libraries(utils_test ${PROJECT_NAME})

ament_add_gtest(graph_test graph_test.cpp)
target_link_libraries(graph_test ${PROJECT_NAME})
