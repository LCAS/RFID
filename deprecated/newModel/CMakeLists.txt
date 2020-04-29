cmake_minimum_required(VERSION 2.8.3)
project(new_models)


#################
## EXTRA FLAGS ##
#################

## mfc for grid_map library 
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

#######################################
##  Find catkin macros and libraries ##
#######################################

find_package( catkin REQUIRED COMPONENTS
  grid_map_core
  grid_map_ros
  grid_map_msgs
  grid_map_cv
  cv_bridge
  sensor_msgs
  nav_msgs
  grid_map_costmap_2d
)

##########################
##  System dependencies ##
##########################

#find_package(Eigen3 REQUIRED)
# Solution to find Eigen3 with Saucy.
find_package(Eigen3 QUIET)
if(NOT EIGEN3_FOUND)
  find_package(PkgConfig REQUIRED)
  pkg_check_modules(EIGEN3 REQUIRED eigen3)
  set(EIGEN3_INCLUDE_DIR ${EIGEN3_INCLUDE_DIRS})
endif()

find_package(yaml-cpp REQUIRED)

find_package(OpenCV REQUIRED
  COMPONENTS
  opencv_highgui
  CONFIG
)

find_package(Boost REQUIRED COMPONENTS random)


###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS grid_map_costmap_2d
  DEPENDS 
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
  dummy 
  src/test_log_model.cpp  
  src/RadarModel.cpp 
)

## Specify libraries to link a library or executable target against
target_link_libraries(
  dummy  
  ${catkin_LIBRARIES}   
  ${OpenCV_LIBRARIES} 
  ${Boost_LIBRARIES}
)

## Mark executable scripts (Python etc.) for installation
install(TARGETS dummy
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
