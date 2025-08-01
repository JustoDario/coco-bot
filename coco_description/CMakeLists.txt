cmake_minimum_required(VERSION 3.8)
project(coco_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

# Install URDF/XACRO files
install(DIRECTORY
  urdf
  launch
  config
  meshes
  DESTINATION share/${PROJECT_NAME}
)

install(FILES
  model.config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
