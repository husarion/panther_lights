cmake_minimum_required(VERSION 2.8.3)
project(panther_lights)

set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  message_generation
)

catkin_python_setup()

# add_message_files(
#   FILES
#   LightsMessage.msg
# )

add_service_files(
  FILES
  Animation.srv
  Brightness.srv
  ImageAnimation.srv
  PanelState.srv
 )

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS rospy message_runtime roscpp
)

include_directories(
  include ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/panther_lights.cpp
# )

# add_executable(lights_controller_simple src/panther_lights/lights_controller_simple.cpp)
# target_link_libraries(lights_controller_simple ${catkin_LIBRARIES})
# add_dependencies(lights_controller_simple panther_lights_gencpp)

# add_executable(lights_controller src/panther_lights/panther_lights.py)
# target_link_libraries(lights_controller ${catkin_LIBRARIES})
# add_dependencies(lights_controller panther_lights_gencpp)


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
