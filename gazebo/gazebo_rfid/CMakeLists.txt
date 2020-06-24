# 2.8.8 required to use PROTOBUF_IMPORT_DIRS
cmake_minimum_required(VERSION 2.8.8 FATAL_ERROR)

project(gazebo_rfid)

find_package(catkin REQUIRED COMPONENTS
  gazebo_dev
  gazebo_plugins
  roscpp
  rfid_node
  sensor_msgs
)

find_package(Boost REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS gazebo_custom_sensor_preloader gazebo_dev gazebo_plugins roscpp sensor_msgs rfid_node
)

include_directories(
  include
)
include_directories(SYSTEM
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  # gazebo msgs
  ${CMAKE_CURRENT_BINARY_DIR}/msgs
)


include_directories(${GAZEBO_INCLUDE_DIRS})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

# TODO: This should create the gazebo msgs
# link_directories(${GAZEBO_LIBRARY_DIRS} ${CMAKE_CURRENT_BINARY_DIR}/msgs)
# add_subdirectory(msgs)

link_directories(${catkin_LIBRARY_DIRS})

# TODO this should create the rfid plugin
# add_library(gazebo_ros_rfid_reader SHARED src/RFIDReaderRosPlugin.cpp)
# add_dependencies(gazebo_ros_rfid_reader ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(gazebo_ros_rfid_reader gazebo_rfid_reader ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${GAZEBO_LIBRARIES})

add_library(gazebo_rfid_reader src/RFIDReader.cpp src/RFIDTransceiver.cpp)
add_dependencies(gazebo_rfid_reader ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(gazebo_rfid_reader ${catkin_LIBRARIES})
target_compile_options(gazebo_rfid_reader PRIVATE -Wall)

add_library(gazebo_rfid_tag src/RFIDTag.cpp src/RFIDTransceiver.cpp)
add_dependencies(gazebo_rfid_tag ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(gazebo_rfid_tag ${catkin_LIBRARIES})
target_compile_options(gazebo_rfid_tag PRIVATE -Wall)



install(TARGETS gazebo_rfid_reader gazebo_rfid_tag
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

install(FILES rfid_reader.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(FILES rfid_tag.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
