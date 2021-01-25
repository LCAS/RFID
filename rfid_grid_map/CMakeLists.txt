cmake_minimum_required(VERSION 2.8.3)
project(rfid_grid_map)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  geometry_msgs
  grid_map_cv
  grid_map_loader
  grid_map_msgs
  grid_map_ros
  grid_map_visualization
  grid_map_costmap_2d
  message_generation
  rfid_node
  roscpp
  sensor_msgs
  tf
)

# Solution to find Eigen3 with Saucy.
find_package(Eigen3 QUIET)
if(NOT EIGEN3_FOUND)
  find_package(PkgConfig REQUIRED)
  pkg_check_modules(EIGEN3 REQUIRED eigen3)
  set(EIGEN3_INCLUDE_DIR ${EIGEN3_INCLUDE_DIRS})
endif()

find_package(OpenCV REQUIRED
  COMPONENTS
  opencv_highgui
  CONFIG
)

find_package(Boost REQUIRED COMPONENTS random)

add_service_files(
   FILES
   GetBeliefMaps.srv
   GetFakeBeliefMaps.srv
)

generate_messages(
  DEPENDENCIES
  grid_map_msgs
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS include
# LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS cv_bridge grid_map_costmap_2d 
# DEPENDS  
)

###########
## Build ##
###########


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${Boost_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIR}
)

## Declare a C++ executable
add_executable(
   rfid_grid_map_node
   src/rfid_grid_map_node.cpp
   src/rfid_gridMap.cpp
   src/RadarModelROS.cpp
   )

## Declare a C++ executable
add_executable(
   viewZois 
   src/viewZois.cpp
   )


## Specify libraries to link a library or executable target against
 target_link_libraries(
   rfid_grid_map_node
   ${catkin_LIBRARIES} 
  ${OpenCV_LIBRARIES} 
  ${Boost_LIBRARIES}
 )


 target_link_libraries(
   viewZois 
   ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES} 
  ${Boost_LIBRARIES}
 )

#############
## Install ##
#############

install(TARGETS rfid_grid_map_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS viewZois
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


## Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE)

install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
  PATTERN ".svn" EXCLUDE)
  
install(DIRECTORY images/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/images
  PATTERN ".svn" EXCLUDE)  

install(DIRECTORY rviz/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/rviz
  PATTERN ".svn" EXCLUDE)
  

#############
## Testing ##
#############
