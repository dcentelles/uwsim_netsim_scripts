cmake_minimum_required(VERSION 2.8.3)
project(uwsim_netsim_scripts)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  uwsim
)

catkin_package(
   INCLUDE_DIRS include
   LIBRARIES uwsim_netsim
   CATKIN_DEPENDS uwsim roscpp rospy std_msgs
#   DEPENDS system_lib
)

###########
## Build ##
###########

SET(${PROJECT_NAME}_CXX_COMPILE_FLAGS "-fPIC -std=c++14 ")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${${PROJECT_NAME}_CXX_COMPILE_FLAGS}")

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# Declare a C++ library
 add_library(${PROJECT_NAME}
   src/SimpleExampleNetSimTracing.cpp
   src/ShowDistanceExampleNetSimTracing.cpp
   src/MoveRobotsNetSimTracing.cpp
   src/StaticRobotsNetSimTracing.cpp
   src/ArdusubTest0.cpp
   src/ShipwreckNetSimTracing.cpp
 )

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

install(DIRECTORY 
  include/${PROJECT_NAME}
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


