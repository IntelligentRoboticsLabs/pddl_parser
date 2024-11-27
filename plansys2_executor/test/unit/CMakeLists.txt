ament_add_gtest(executor_test executor_test.cpp TIMEOUT 300)
target_compile_definitions(executor_test
  PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")
target_link_libraries(executor_test ${PROJECT_NAME})

ament_add_gtest(action_execution_test action_execution_test.cpp)
target_compile_definitions(action_execution_test
  PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")
target_link_libraries(action_execution_test ${PROJECT_NAME})


ament_add_gtest(execution_tree_test execution_tree_test.cpp)
target_compile_definitions(execution_tree_test
  PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")
target_link_libraries(execution_tree_test ${PROJECT_NAME})
ament_target_dependencies(execution_tree_test ${dependencies})

ament_add_gtest(simple_btbuilder_tests simple_btbuilder_tests.cpp TIMEOUT 1200)
target_compile_definitions(simple_btbuilder_tests
  PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")
target_link_libraries(simple_btbuilder_tests ${PROJECT_NAME} bt_builder_plugins)
ament_target_dependencies(simple_btbuilder_tests ${dependencies})

ament_add_gtest(bt_node_test bt_node_test.cpp)
target_compile_definitions(bt_node_test
  PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")
target_link_libraries(bt_node_test ${PROJECT_NAME})
ament_target_dependencies(bt_node_test ${dependencies})

ament_add_gtest(bt_node_test_charging bt_node_test_charging.cpp)
target_compile_definitions(bt_node_test_charging
  PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")
target_link_libraries(bt_node_test_charging ${PROJECT_NAME})
ament_target_dependencies(bt_node_test_charging ${dependencies})

ament_add_gtest(bt_node_test_suave bt_node_test_suave.cpp)
target_compile_definitions(bt_node_test_suave
  PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")
target_link_libraries(bt_node_test_suave ${PROJECT_NAME})
ament_target_dependencies(bt_node_test_suave ${dependencies})
