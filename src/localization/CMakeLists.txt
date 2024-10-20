cmake_minimum_required(VERSION 3.0.2)
project(localization)


## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS 
  rospy
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  ${catkin_INCLUDE_DIRS}
)

#############
## Install ##
#############

catkin_package(
  CATKIN_DEPENDS rospy
)

## Mark executable scripts (Python etc.) for installationss
catkin_python_setup()
catkin_package()

catkin_install_python(PROGRAMS
  src/transform_broadcaster.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)