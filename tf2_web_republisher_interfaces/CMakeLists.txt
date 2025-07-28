cmake_minimum_required(VERSION 3.22)
project(tf2_web_republisher_interfaces)

find_package(ament_cmake REQUIRED)

set(THIS_PACKAGE_INCLUDE_DEPENDS
    builtin_interfaces
    geometry_msgs
    rosidl_default_generators
    std_msgs
    tf2_eigen
    tf2_geometry_msgs
    tf2_msgs)

foreach(package IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${package} REQUIRED)
endforeach()

rosidl_generate_interfaces(
  ${PROJECT_NAME}
  msg/TFArray.msg
  services/RepublishTFs.srv
  action/TFSubscription.action
  DEPENDENCIES
  builtin_interfaces
  action_msgs
  geometry_msgs
  std_msgs
  tf2_msgs
  builtin_interfaces)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
