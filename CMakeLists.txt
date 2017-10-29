cmake_minimum_required(VERSION 2.8.3)
project(gazebo_ar)



find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  visualization_msgs   
  ar_track_alvar
  roscpp
  tf
)

find_package(gazebo REQUIRED)

#catkin_package(CATKIN_DEPENDS message_runtime ar_track_alvar_msgs visualization_msgs)
#add_message_files(FILES AlvarMarker.msg AlvarMarkers.msg)
#necessary if I want to use the alvarmarker messages
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)

link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

add_executable(translator src/translator.cc)
target_link_libraries(translator ${GAZEBO_LIBRARIES} pthread)
target_link_libraries(translator ${catkin_LIBRARIES})















