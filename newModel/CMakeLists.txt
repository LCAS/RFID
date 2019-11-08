cmake_minimum_required(VERSION 2.8.3)
project(new_models)

include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")


find_package( catkin REQUIRED COMPONENTS
  grid_map_core
  grid_map_cv
  cv_bridge  
)

## System dependencies are found with CMake's conventions
#find_package(Eigen3 REQUIRED)
# Solution to find Eigen3 with Saucy.
find_package(Eigen3 QUIET)
if(NOT EIGEN3_FOUND)
  find_package(PkgConfig REQUIRED)
  pkg_check_modules(EIGEN3 REQUIRED eigen3)
  set(EIGEN3_INCLUDE_DIR ${EIGEN3_INCLUDE_DIRS})
endif()


include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

find_package(OpenCV REQUIRED
  COMPONENTS
  opencv_highgui
  CONFIG
)

## ............................................................................

#Find all the files link them to the executable (for qtcreator)
FILE(GLOB_RECURSE HEADER_FILES include *.hpp)

add_executable(dummy src/test_log_model.cpp  src/RadarModel.cpp ${HEADER_FILES})
target_link_libraries(dummy  ${catkin_LIBRARIES}   ${OpenCV_LIBRARIES})
install(TARGETS dummy RUNTIME DESTINATION bin)
