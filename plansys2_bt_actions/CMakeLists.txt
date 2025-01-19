cmake_minimum_required(VERSION 3.5)

project(plansys2_bt_actions)

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(plansys2_executor REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(action_msgs REQUIRED)
find_package(lifecycle_msgs REQUIRED)



set(dependencies
    rclcpp
    rclcpp_action
    rclcpp_lifecycle
    plansys2_executor
    behaviortree_cpp
    action_msgs
    lifecycle_msgs
)

include_directories(include)

set(BT_ACTIONS_SOURCES
  src/plansys2_bt_actions/BTAction.cpp
)

add_library(${PROJECT_NAME} SHARED ${BT_ACTIONS_SOURCES})
ament_target_dependencies(${PROJECT_NAME} ${dependencies})

add_executable(bt_action_node
  src/bt_action_node.cpp
)
ament_target_dependencies(bt_action_node ${dependencies})
target_link_libraries(bt_action_node ${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include/
)

install(TARGETS
  bt_action_node
  ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  find_package(plansys2_msgs REQUIRED)
  find_package(test_msgs REQUIRED)
  find_package(geometry_msgs REQUIRED)

  set(dependencies ${dependencies} plansys2_msgs test_msgs geometry_msgs)

  add_subdirectory(test)
endif()

ament_export_include_directories(include)
ament_export_dependencies(${dependencies})

ament_package()