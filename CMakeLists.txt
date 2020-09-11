cmake_minimum_required(VERSION 2.8.3)
project(ecn_sensorbased)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_BUILD_TYPE Debug)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
 cmake_modules
  cv_bridge
  roscpp
  sensor_msgs
  geometry_msgs
  image_transport
  ecn_common
)

## System dependencies are found with CMake's conventions
find_package(OpenCV REQUIRED)
find_package(VISP REQUIRED)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include ${OpenCV_INCLUDE_DIRS}  ${VISP_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS}
  LIBRARIES ecn_sensorbased
  CATKIN_DEPENDS cv_bridge roscpp sensor_msgs geometry_msgs image_transport ecn_common
  DEPENDS ${OpenCV_LIBRARIES}  ${VISP_LIBRARIES}
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${VISP_INCLUDE_DIRS}
)

add_library(ecn_sensorbased include/ecn_sensorbased/pioneer_cam.h src/pioneer_cam.cpp)
target_link_libraries(ecn_sensorbased ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${VISP_LIBRARIES})

add_executable(pioneer  src/main.cpp)
target_link_libraries(pioneer ecn_sensorbased)
