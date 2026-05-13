cmake_minimum_required(VERSION 3.10)
project(aruco_imu_eskf_localization_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_AUTOMOC ON)
add_definitions(-DQT_NO_KEYWORDS)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc calib3d aruco)
find_package(pluginlib REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Core Widgets)
find_package(rclcpp REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_ogre_vendor REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

include_directories(include)

set(core_sources
  src/board_pose_estimator.cpp
  src/camera_calibration.cpp
  src/diagnostics_publisher.cpp
  src/geometry.cpp
  src/gyro_relative_eskf.cpp
)

add_library(${PROJECT_NAME}_core SHARED ${core_sources})
target_include_directories(${PROJECT_NAME}_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME}_core Eigen3::Eigen ${OpenCV_LIBS} yaml-cpp)
ament_target_dependencies(${PROJECT_NAME}_core
  ament_index_cpp
  diagnostic_msgs
  geometry_msgs
  nav_msgs
  rclcpp
  sensor_msgs
  tf2
  tf2_eigen
  tf2_ros
)

add_executable(aruco_detector_node src/nodes/aruco_detector_node.cpp)
target_link_libraries(aruco_detector_node ${PROJECT_NAME}_core)
ament_target_dependencies(aruco_detector_node
  ament_index_cpp
  cv_bridge
  diagnostic_msgs
  geometry_msgs
  rclcpp
  sensor_msgs
  tf2_ros
)

add_executable(relative_localization_node src/nodes/relative_localization_node.cpp)
target_link_libraries(relative_localization_node ${PROJECT_NAME}_core)
ament_target_dependencies(relative_localization_node
  diagnostic_msgs
  geometry_msgs
  nav_msgs
  rclcpp
  sensor_msgs
  tf2_ros
)

add_executable(pose_rviz_marker_node src/nodes/pose_rviz_marker_node.cpp)
target_link_libraries(pose_rviz_marker_node ${PROJECT_NAME}_core)
ament_target_dependencies(pose_rviz_marker_node
  geometry_msgs
  nav_msgs
  rclcpp
  std_msgs
  visualization_msgs
)

set(overlay_headers_to_moc
  include/aruco_imu_eskf_localization_cpp/rviz_overlay/aruco_imu_eskf_debug_display.hpp
)
foreach(header "${overlay_headers_to_moc}")
  qt5_wrap_cpp(overlay_moc_files "${header}")
endforeach()

add_library(aruco_imu_eskf_debug_rviz_plugin SHARED
  ${overlay_moc_files}
  src/rviz_overlay/aruco_imu_eskf_debug_display.cpp
  src/rviz_overlay/overlay_utils.cpp
)
target_include_directories(aruco_imu_eskf_debug_rviz_plugin PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${Qt5Widgets_INCLUDE_DIRS}
)
target_link_libraries(aruco_imu_eskf_debug_rviz_plugin
  rviz_ogre_vendor::OgreMain
  rviz_ogre_vendor::OgreOverlay
  Qt5::Widgets
)
ament_target_dependencies(aruco_imu_eskf_debug_rviz_plugin
  diagnostic_msgs
  nav_msgs
  rclcpp
  rviz_common
  rviz_rendering
  sensor_msgs
)
target_compile_definitions(aruco_imu_eskf_debug_rviz_plugin PRIVATE "ARUCO_IMU_ESKF_DEBUG_RVIZ_PLUGIN_BUILDING_LIBRARY")
target_compile_definitions(aruco_imu_eskf_debug_rviz_plugin PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

pluginlib_export_plugin_description_file(rviz_common plugins_description.xml)

install(TARGETS
  ${PROJECT_NAME}_core
  aruco_detector_node
  relative_localization_node
  pose_rviz_marker_node
  aruco_imu_eskf_debug_rviz_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)
install(FILES plugins_description.xml
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_gyro_relative_eskf test/test_gyro_relative_eskf.cpp)
  target_link_libraries(test_gyro_relative_eskf ${PROJECT_NAME}_core)

  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}_core)
ament_package()
