cmake_minimum_required(VERSION 3.22)
project(tf2_web_republisher)

find_package(ament_cmake REQUIRED)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  geometry_msgs
  rclcpp
  rclcpp_action
  rclcpp_components
  tf2
  tf2_ros
  tf2_web_republisher_interfaces
)

foreach(package IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${package} REQUIRED)
endforeach()

add_library(${PROJECT_NAME} SHARED
  src/tf2_web_republisher.cpp
)
target_include_directories(
  ${PROJECT_NAME}
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include>)

target_link_libraries(tf2_web_republisher PUBLIC
  ${builtin_interfaces_TARGETS}
  ${tf2_msgs_TARGETS}
  ${tf2_web_republisher_interfaces_TARGETS}
  rclcpp::rclcpp
  rclcpp_action::rclcpp_action
  rclcpp_components::component
  tf2::tf2
  tf2_ros::tf2_ros
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "tf2_web_republisher::TFRepublisher"
  EXECUTABLE tf2_web_republisher_node
)

if(BUILD_TESTING)
  add_subdirectory(test)
endif()

install(DIRECTORY include/ DESTINATION include)
install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
