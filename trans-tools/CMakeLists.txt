cmake_minimum_required(VERSION 2.8.3)
project(trans_tools)

set(CMAKE_BUILD_TYPE Release)
set(BUILD_STATIC_LIBS ON)
set(BUILD_SHARED_LIBS OFF)
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  pcl_ros
)

find_package(PCL REQUIRED)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  CATKIN_DEPENDS
  roscpp
  std_msgs
  sensor_msgs
  pcl_ros
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  /usr/local/include # Location when using 'make system_install'
  /usr/include       # More usual location (e.g. when installing using a package)
)

## Specify additional locations for library files
link_directories(
  /usr/local/lib # Location when using 'make system_install'
  /usr/lib       # More usual location (e.g. when installing using a package)
)

## Declare executables
add_executable(odometry2kitti src/odometry2kitti.cpp)
add_executable(data2value src/data2value.cpp)
add_executable(savelabelmap src/savelabelmap.cpp)
add_executable(extractSeg src/extractSeg.cpp)
add_executable(cluster src/cluster.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(odometry2kitti ${catkin_LIBRARIES})
target_link_libraries(data2value ${catkin_LIBRARIES})
target_link_libraries(savelabelmap ${catkin_LIBRARIES} ${PCL_LIBRARIES})
target_link_libraries(extractSeg ${catkin_LIBRARIES} ${PCL_LIBRARIES})
target_link_libraries(cluster ${catkin_LIBRARIES} ${PCL_LIBRARIES})
