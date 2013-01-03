cmake_minimum_required(VERSION 2.8.3)
project(tf2_web_republisher)

find_package(catkin REQUIRED COMPONENTS tf tf2_ros geometry_msgs actionlib actionlib_msgs)
find_package(Boost COMPONENTS thread REQUIRED)

add_action_files(DIRECTORY action FILES TFSubscription.action)
generate_messages(
  DEPENDENCIES actionlib_msgs std_msgs geometry_msgs
)


include_directories(/include)
include_directories(${catkin_INCLUDE_DIRS})

add_executable(${PROJECT_NAME} src/tf_web_republisher.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES}) 
add_dependencies(${PROJECT_NAME} actionlib_gencpp)

add_executable(${PROJECT_NAME}_test_subscription src/test_subscription.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES}) 
add_dependencies(${PROJECT_NAME} actionlib_gencpp)

catkin_package(
    CATKIN_DEPENDS tf tf2_ros geometry_msgs actionlib actionlib_msgs
)


