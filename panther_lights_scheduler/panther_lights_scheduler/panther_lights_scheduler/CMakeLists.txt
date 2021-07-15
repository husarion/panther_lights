cmake_minimum_required(VERSION 2.8.3)
project(panther_lights_scheduler)

find_package(Threads REQUIRED)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  rospy
  husarion_msgs
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS
  rospy
  husarion_msgs
  message_runtime
)


install(PROGRAMS
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


# install(FILES
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
# )
