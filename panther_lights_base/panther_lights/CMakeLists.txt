cmake_minimum_required(VERSION 2.8.3)
project(panther_lights)

set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
)

catkin_python_setup()

add_service_files(
  FILES
  Brightness.srv
  PanelState.srv
 )

add_message_files(
  FILES
  Animation.msg
  ImageAnimation.msg
)


generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS rospy message_runtime
)

if (CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(tests/config_import_test.py)
endif()


#############
## Install ##
#############

install(PROGRAMS
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


## Mark other files for installation (e.g. launch and bag files, etc.)
install(FILES
  config/led_conf.yaml
  config/panther_events.yaml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)
