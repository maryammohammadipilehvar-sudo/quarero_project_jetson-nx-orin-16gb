cmake_minimum_required(VERSION 3.5)
project(interfaces)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
    "msg/Status.msg"
    "msg/CommandDrive.msg"
    "msg/Joy.msg"
    "msg/GeoPath.msg"
    "msg/Schedule.msg"
    "msg/SecurityAlert.msg"
    "msg/SectorInfo.msg"
    "msg/ObstacleSectors.msg"
    "srv/CommandControl.srv"
    "srv/WaypointService.srv"
    "srv/ScheduleService.srv"
    "srv/CaptureEventClips.srv"
    "srv/GetScheduleAction.srv"
    DEPENDENCIES geometry_msgs builtin_interfaces std_msgs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
