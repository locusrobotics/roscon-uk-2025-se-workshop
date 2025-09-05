cmake_minimum_required(VERSION 3.8)
project(task8)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(fuse_constraints REQUIRED)
find_package(fuse_core REQUIRED)
find_package(fuse_models REQUIRED)
find_package(fuse_variables REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(workshop_msgs REQUIRED)

add_library(${PROJECT_NAME}
  src/range_constraint.cpp
  src/beacon_sensor_model.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
target_link_libraries(${PROJECT_NAME} PUBLIC
  fuse_constraints::fuse_constraints
  fuse_core::fuse_core
  fuse_models::fuse_models
  fuse_variables::fuse_variables
  ${geometry_msgs_TARGETS}
  rclcpp::rclcpp
  ${workshop_msgs_TARGETS}
)

add_executable(beacon_node src/beacon_node.cpp)
target_link_libraries(
  beacon_node PUBLIC
    rclcpp::rclcpp
    ${geometry_msgs_TARGETS}
    ${nav_msgs_TARGETS}
    ${std_msgs_TARGETS}
    ${visualization_msgs_TARGETS}
    ${workshop_msgs_TARGETS})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(TARGETS ${PROJECT_NAME} EXPORT ${PROJECT_NAME}-export
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
  beacon_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

# If we install a directory, --symlink-install doesn't work
install(FILES
  config/two_tier.yaml
  config/two_tier.rviz
  DESTINATION share/${PROJECT_NAME}/config
)

install(FILES
  launch/fls.launch.xml
  DESTINATION share/${PROJECT_NAME}/launch
)

pluginlib_export_plugin_description_file(fuse_core fuse_plugins.xml)

ament_export_targets(${PROJECT_NAME}-export HAS_LIBRARY_TARGET)
ament_export_dependencies(
  ament_cmake_ros
  fuse_constraints
  fuse_core
  fuse_models
  fuse_variables
  geometry_msgs
  rclcpp
  workshop_msgs
)

ament_package()
