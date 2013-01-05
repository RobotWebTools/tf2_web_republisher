cmake_minimum_required(VERSION 2.8.3)
project(tf2_web_republisher)

find_package(catkin REQUIRED COMPONENTS tf tf2_ros geometry_msgs actionlib actionlib_msgs roscpp)
find_package(Boost REQUIRED COMPONENTS thread )

add_action_files(DIRECTORY action FILES TFSubscription.action)
generate_messages(
  DEPENDENCIES actionlib_msgs std_msgs geometry_msgs roscpp
) 

catkin_package(
    CATKIN_DEPENDS tf tf2_ros geometry_msgs actionlib actionlib_msgs
)

include_directories(include)
include_directories(${catkin_INCLUDE_DIRS})

add_executable(${PROJECT_NAME} src/tf_web_republisher.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES}) 
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencpp)

add_executable(${PROJECT_NAME}_test_subscription tests/test_subscription.cpp)
target_link_libraries(${PROJECT_NAME}_test_subscription ${catkin_LIBRARIES} ${Boost_LIBRARIES}) 
add_dependencies(${PROJECT_NAME}_test_subscription ${PROJECT_NAME}_gencpp)

add_executable(${PROJECT_NAME}_test_subscription tests/test_subscription2.cpp)
target_link_libraries(${PROJECT_NAME}_test_subscription ${catkin_LIBRARIES} ${Boost_LIBRARIES}) 
add_dependencies(${PROJECT_NAME}_test_subscription ${PROJECT_NAME}_gencpp)


install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_test_subscription
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)



