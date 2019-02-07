cmake_minimum_required(VERSION 2.8.3)
project(ar_tf)



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
catkin_package()

link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

add_executable(single_tag src/single_tag.cpp)
target_link_libraries(single_tag ${GAZEBO_LIBRARIES} pthread)
target_link_libraries(single_tag ${catkin_LIBRARIES})

add_executable(multiple_tags src/multiple_tags.cpp)
target_link_libraries(multiple_tags ${GAZEBO_LIBRARIES} pthread)
target_link_libraries(multiple_tags ${catkin_LIBRARIES})













