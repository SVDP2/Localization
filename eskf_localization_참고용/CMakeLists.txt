cmake_minimum_required(VERSION 3.10)
project(eskf_localization)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# C++17 required
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(std_srvs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(autoware_vehicle_msgs REQUIRED)
find_package(tier4_map_msgs REQUIRED)
find_package(geographic_msgs REQUIRED)
find_package(geography_utils REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(kiss_icp REQUIRED)

find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)

# KISS-ICP transitive dependencies (exported as CMake targets)
# Force CONFIG mode to avoid picking up non-standard FindTBB modules.
find_package(TBB REQUIRED CONFIG)
find_package(Sophus REQUIRED CONFIG)

# RViz plugin dependencies
find_package(Qt5 REQUIRED COMPONENTS Core Widgets)
find_package(rviz_common REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(rviz_ogre_vendor REQUIRED)
find_package(pluginlib REQUIRED)

# Include directories
include_directories(include)

# ESKF Localization Node executable
add_executable(eskf_localization_node
  src/main.cpp
  src/parameters.cpp
  src/node/core.cpp
  src/node/params.cpp
  src/node/cb_imu.cpp
  src/node/cb_gnss.cpp
  src/node/cb_vehicle.cpp
  src/node/cb_initialpose.cpp
  src/node/cb_pointcloud.cpp
  src/node/publish.cpp
  src/eskf/eskf_core.cpp
  src/eskf/gnss_update_handler.cpp
  src/eskf/odom_builder.cpp
  src/preprocess/gnss_heading_arbitrator.cpp
  src/preprocess/gnss_preprocessor.cpp
  src/preprocess/imu_calibration_manager.cpp
  src/preprocess/imu_preprocessor.cpp
  src/util/map_projector.cpp
  src/util/tf_cache.cpp
  src/util/diagnostics_publisher.cpp
)

ament_target_dependencies(eskf_localization_node
  ament_index_cpp
  rclcpp
  sensor_msgs
  nav_msgs
  diagnostic_msgs
  geometry_msgs
  tf2_ros
  tf2_geometry_msgs
  std_srvs
  autoware_vehicle_msgs
  tier4_map_msgs
  geographic_msgs
  geography_utils
  std_msgs
  kiss_icp
)

target_include_directories(eskf_localization_node SYSTEM PRIVATE ${EIGEN3_INCLUDE_DIR})
target_link_libraries(eskf_localization_node Eigen3::Eigen)
target_link_libraries(eskf_localization_node kiss_icp::kiss_icp_pipeline)

# IMU Dead Reckoning Node executable
add_executable(imu_dead_reckoning
  src/debug/imu_dead_reckoning_main.cpp
  src/debug/imu_dead_reckoning_node.cpp
  src/preprocess/gnss_preprocessor.cpp
  src/util/map_projector.cpp
  src/util/tf_cache.cpp
)

ament_target_dependencies(imu_dead_reckoning
  rclcpp
  sensor_msgs
  geometry_msgs
  tf2_ros
  tf2_geometry_msgs
  tier4_map_msgs
  geographic_msgs
  geography_utils
)

target_include_directories(imu_dead_reckoning SYSTEM PRIVATE ${EIGEN3_INCLUDE_DIR})
target_link_libraries(imu_dead_reckoning Eigen3::Eigen)

# OBD Dead Reckoning Node executable
add_executable(obd_dead_reckoning
  src/debug/obd_dead_reckoning_main.cpp
  src/debug/obd_dead_reckoning_node.cpp
)

ament_target_dependencies(obd_dead_reckoning
  rclcpp
  geometry_msgs
  tf2_ros
  tf2_geometry_msgs
  autoware_vehicle_msgs
)


# ESKF Dead Reckoning Node executable (lite shadow using EskfCore, without GNSS updates)
add_executable(eskf_dead_reckoning
  src/debug/eskf_dead_reckoning_main.cpp
  src/debug/eskf_dead_reckoning_node.cpp
  src/eskf/eskf_core.cpp
)

ament_target_dependencies(eskf_dead_reckoning
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  tf2_ros
  tf2_geometry_msgs
  autoware_vehicle_msgs
)

target_include_directories(eskf_dead_reckoning SYSTEM PRIVATE ${EIGEN3_INCLUDE_DIR})
target_link_libraries(eskf_dead_reckoning Eigen3::Eigen)

# Qt MOC setup for RViz plugin
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOURC ON)
add_definitions(-DQT_NO_KEYWORDS)

set(headers_to_moc
  include/eskf_localization/rviz_overlay/gnss_debug_display.hpp
)

foreach(header "${headers_to_moc}")
  qt5_wrap_cpp(gnss_debug_moc_files "${header}")
endforeach()

# GNSS Debug RViz Plugin library
add_library(gnss_debug_rviz_plugin SHARED
  ${gnss_debug_moc_files}
  src/rviz_overlay/gnss_debug_display.cpp
  src/rviz_overlay/gnss_data_helper.cpp
  src/rviz_overlay/localization_data_helper.cpp
  src/rviz_overlay/vehicle_data_helper.cpp
  src/rviz_overlay/imu_data_helper.cpp
  src/rviz_overlay/status_chart_helper.cpp
  src/rviz_overlay/update_status_helper.cpp
  src/rviz_overlay/overlay_utils.cpp
)

set_property(TARGET gnss_debug_rviz_plugin PROPERTY CXX_STANDARD 17)
target_compile_options(gnss_debug_rviz_plugin PRIVATE -Wall -Wextra -Wpedantic)

target_include_directories(gnss_debug_rviz_plugin PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${Qt5Widgets_INCLUDE_DIRS}
)

target_link_libraries(gnss_debug_rviz_plugin
  rviz_ogre_vendor::OgreMain
  rviz_ogre_vendor::OgreOverlay
  Qt5::Widgets
)

ament_target_dependencies(gnss_debug_rviz_plugin
  rviz_common
  rviz_rendering
  sensor_msgs
  geometry_msgs
  nav_msgs
  diagnostic_msgs
  std_msgs
  autoware_vehicle_msgs
  rclcpp
)

target_compile_definitions(gnss_debug_rviz_plugin PRIVATE "gnss_debug_rviz_plugin_BUILDING_LIBRARY")
target_compile_definitions(gnss_debug_rviz_plugin PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

pluginlib_export_plugin_description_file(rviz_common plugins_description.xml)

# Install executables
install(TARGETS
  eskf_localization_node
  imu_dead_reckoning
  obd_dead_reckoning
  eskf_dead_reckoning
  DESTINATION lib/${PROJECT_NAME}
)

# Install RViz plugin library
install(
  TARGETS gnss_debug_rviz_plugin
  EXPORT gnss_debug_rviz_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

# Install header files
install(DIRECTORY include/
  DESTINATION include
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install config files (for calibration, etc.)
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
