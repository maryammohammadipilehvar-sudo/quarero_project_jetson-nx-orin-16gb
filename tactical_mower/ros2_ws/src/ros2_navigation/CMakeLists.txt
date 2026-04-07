cmake_minimum_required(VERSION 3.8)
project(ros2_navigation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies for message generation
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(Python3 COMPONENTS Interpreter REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# Generate services
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/ComputePathToPose.srv"
  "srv/ReplanPath.srv"
  DEPENDENCIES geometry_msgs nav_msgs std_msgs
)

# Install Python modules manually (without setup.py to avoid conflicts)
# Find Python version for install path
execute_process(
  COMMAND ${Python3_EXECUTABLE} -c "import sys; print(f'{sys.version_info.major}.{sys.version_info.minor}')"
  OUTPUT_VARIABLE PYTHON_VERSION
  OUTPUT_STRIP_TRAILING_WHITESPACE
)

# Install Python package directory to ROS2 standard location
install(DIRECTORY ros2_navigation
  DESTINATION lib/python${PYTHON_VERSION}/site-packages
  FILES_MATCHING
  PATTERN "*.py"
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)

# Install entry point script
install(PROGRAMS
  ${CMAKE_CURRENT_SOURCE_DIR}/ros2_navigation/nav2_navigation_node.py
  RENAME nav2_navigation_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install config, launch and rviz files
install(DIRECTORY config launch rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()

