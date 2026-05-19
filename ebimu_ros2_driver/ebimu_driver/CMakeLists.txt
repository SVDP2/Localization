cmake_minimum_required(VERSION 3.8)
project(ebimu_driver)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)
find_package(diagnostic_msgs REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(ebimu_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)

include_directories(include)

add_executable(ebimu_node
  src/ebimu_node.cpp
  src/ebimu_parser.cpp
  src/main.cpp
  src/serial_port.cpp
)

ament_target_dependencies(ebimu_node
  diagnostic_msgs
  diagnostic_updater
  ebimu_interfaces
  rclcpp
  sensor_msgs
  std_msgs
  tf2
)
target_link_libraries(ebimu_node Boost::system)

install(TARGETS ebimu_node
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/ebimu_tui.py
  scripts/ebimu_serial_sim.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
