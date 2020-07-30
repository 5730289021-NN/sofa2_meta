cmake_minimum_required(VERSION 2.8.3)
project(sofa2_dynamixel)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
)

# protected region additional user defined REQUIREMENTS begin #
# protected region additional user defined REQUIREMENTS end #


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS
    common/src
  CATKIN_DEPENDS
    roscpp
    sensor_msgs
  DEPENDS
    async_comm
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(common/src)
include_directories(${catkin_INCLUDE_DIRS})

## Declare cpp executable
add_executable(dynamixel_controller ros/src/dynamixel_controller_ros.cpp)
## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
## Specify libraries to link a library or executable target against
target_link_libraries(dynamixel_controller ${catkin_LIBRARIES} ${async_comm_LIBRARIES}
)
# protected region additional user defined BUILD STATEMENTS begin #
# protected region additional user defined BUILD STATEMENTS end #

##########
## Test ##
##########

if (CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest(test/test_dynamixel_controller_hz_loop.test)
endif()

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS dynamixel_controller
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# protected region user Cmake macros begin #
# protected region user Cmake macros end #
