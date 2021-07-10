cmake_minimum_required(VERSION 2.8.3)
project(panther_lights_scheduler)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  message_generation
)

catkin_python_setup()


catkin_package(
  CATKIN_DEPENDS rospy message_runtime
)

install(PROGRAMS
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
