cmake_minimum_required(VERSION 3.8)
project(coco_mov_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(control_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)


set(dependencies
  rclcpp
  rclcpp_action
  control_msgs
  trajectory_msgs
  geometry_msgs
)

include_directories(include)

add_library(${PROJECT_NAME} SHARED
  src/coco_mov_control/gait_planifier.cpp
)
ament_target_dependencies(${PROJECT_NAME} ${dependencies})


add_executable(gait_planifier src/gait_planifier_main.cpp)
ament_target_dependencies(gait_planifier_main ${dependencies})
target_link_libraries(gait_planifier_main ${PROJECT_NAME})


install(TARGETS
  ${PROJECT_NAME}
  gait_planifier_main
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)

  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  add_subdirectory(test)
endif()

ament_package()
