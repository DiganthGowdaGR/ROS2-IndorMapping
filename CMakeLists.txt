cmake_minimum_required(VERSION 3.8)
project(indoor_bot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/PointArray.msg"
  DEPENDENCIES
    geometry_msgs
    std_msgs
)

install(
  DIRECTORY
    config
    launch
    maps
    models
    rviz
    worlds
  DESTINATION share/${PROJECT_NAME}
)

set(PROJECT_DOC_FILES
  README.md
)

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/LICENSE.md")
  list(APPEND PROJECT_DOC_FILES LICENSE.md)
endif()

install(
  FILES
    ${PROJECT_DOC_FILES}
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/drive_square.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME drive_square
)

install(
  PROGRAMS
    scripts/frontier_explorer.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME frontier_explorer
)

install(
  PROGRAMS
    scripts/save_map.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME save_map
)

install(
  PROGRAMS
    scripts/scan_motion_gate.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME scan_motion_gate
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
