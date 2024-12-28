cmake_minimum_required(VERSION 3.0.2)
project(robot_delivery_system)


find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  geometry_msgs
  move_base
  nav_msgs
  roscpp
  rospy
  std_msgs
)

## Generate messages in the 'msg' folder
add_message_files(
   FILES
   OrderList.msg

)

## Generate services in the 'srv' folder
add_service_files(
   FILES
   PopupService.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   actionlib_msgs   geometry_msgs   nav_msgs   std_msgs
)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES robot_delivery_system
#  CATKIN_DEPENDS actionlib actionlib_msgs geometry_msgs move_base nav_msgs roscpp rospy std_msgs
#  DEPENDS system_lib
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)



add_executable(test_node src/test_node.cpp)

add_dependencies(test_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(test_node ${catkin_LIBRARIES})


add_executable(order_manager_node
  src/main.cpp
  src/robot_movement.cpp
  src/order_manager.cpp
)

add_dependencies(order_manager_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(order_manager_node ${catkin_LIBRARIES})

catkin_install_python(PROGRAMS
  scripts/popup_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
