cmake_minimum_required(VERSION 3.5)

project(plansys2_terminal)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(plansys2_msgs REQUIRED)
find_package(plansys2_domain_expert REQUIRED)
find_package(plansys2_problem_expert REQUIRED)
find_package(plansys2_planner REQUIRED)
find_package(plansys2_executor REQUIRED)
find_package(plansys2_pddl_parser REQUIRED)



set(dependencies
    rclcpp
    rclcpp_action
    rclcpp_lifecycle
    plansys2_msgs
    plansys2_domain_expert
    plansys2_problem_expert
    plansys2_planner
    plansys2_executor
    plansys2_pddl_parser
)

include_directories(include)

add_library(terminal SHARED
  src/plansys2_terminal/Terminal.cpp)
ament_target_dependencies(terminal ${dependencies})
target_compile_definitions(terminal PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

add_executable(${PROJECT_NAME}
  src/terminal_node.cpp)
ament_target_dependencies(${PROJECT_NAME} ${dependencies})
target_link_libraries(${PROJECT_NAME} terminal readline)

install(TARGETS
  terminal
  ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_index_cpp REQUIRED)
  find_package(lifecycle_msgs REQUIRED)

  set(dependencies ${dependencies} ament_index_cpp lifecycle_msgs)

  add_subdirectory(test)
endif()

ament_package()
