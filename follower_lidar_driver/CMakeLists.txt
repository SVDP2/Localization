cmake_minimum_required(VERSION 3.10)
project(follower_lidar_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)

find_package(ydlidar_sdk QUIET)

if(ydlidar_sdk_FOUND)
  add_executable(follower_lidar_node src/follower_lidar_node.cpp)
  target_include_directories(follower_lidar_node PRIVATE
    ${YDLIDAR_SDK_INCLUDE_DIRS}
  )
  target_link_directories(follower_lidar_node PRIVATE
    ${YDLIDAR_SDK_LIBRARY_DIRS}
  )
  target_link_libraries(follower_lidar_node
    ${YDLIDAR_SDK_LIBRARIES}
  )
  ament_target_dependencies(follower_lidar_node
    geometry_msgs
    rclcpp
    sensor_msgs
    std_srvs
  )

  install(TARGETS follower_lidar_node
    DESTINATION lib/${PROJECT_NAME}
  )
else()
  message(WARNING
    "ydlidar_sdk was not found. follower_lidar_node will not be built; "
    "install YDLidar-SDK on the target robot to enable this package.")
endif()

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
