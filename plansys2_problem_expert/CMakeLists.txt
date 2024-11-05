cmake_minimum_required(VERSION 3.5)

project(plansys2_problem_expert)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(plansys2_pddl_parser REQUIRED)
find_package(plansys2_core REQUIRED)
find_package(plansys2_msgs REQUIRED)
find_package(plansys2_domain_expert REQUIRED)
find_package(std_msgs REQUIRED)
find_package(lifecycle_msgs REQUIRED)

find_package(OpenMP REQUIRED)

set(CMAKE_CXX_STANDARD 17)

set(dependencies
    rclcpp
    rclcpp_action
    rclcpp_lifecycle
    plansys2_pddl_parser
    ament_index_cpp
    plansys2_core
    plansys2_msgs
    plansys2_domain_expert
    std_msgs
    lifecycle_msgs
    OpenMP
)

include_directories(include)

set(PROBLEM_EXPERT_SOURCES
  src/plansys2_problem_expert/ProblemExpert.cpp
  src/plansys2_problem_expert/ProblemExpertClient.cpp
  src/plansys2_problem_expert/ProblemExpertNode.cpp
  src/plansys2_problem_expert/Utils.cpp
)

add_library(${PROJECT_NAME} SHARED ${PROBLEM_EXPERT_SOURCES})
ament_target_dependencies(${PROJECT_NAME} ${dependencies})

# # Link OpenMP to the target
if(OpenMP_CXX_FOUND)
  target_link_libraries(${PROJECT_NAME} OpenMP::OpenMP_CXX)
endif()

add_executable(problem_expert_node
  src/problem_expert_node.cpp
)
ament_target_dependencies(problem_expert_node ${dependencies})
target_link_libraries(problem_expert_node ${PROJECT_NAME})

if(OpenMP_CXX_FOUND)
  target_link_libraries(problem_expert_node OpenMP::OpenMP_CXX)
endif()

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

install(TARGETS
  ${PROJECT_NAME}
  problem_expert_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  add_subdirectory(test)
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(${dependencies})

ament_package()
