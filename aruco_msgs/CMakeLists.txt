cmake_minimum_required(VERSION 2.8.3)
project(aruco_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  geometry_msgs
  std_msgs
  sensor_msgs)
  

add_message_files( FILES Marker.msg MarkerArray.msg )
add_service_files(
  FILES
  MarkerReq.srv
  MarkersReq.srv
)
generate_messages( DEPENDENCIES std_msgs geometry_msgs sensor_msgs)

catkin_package(
  CATKIN_DEPENDS
    message_runtime
    geometry_msgs
    std_msgs
)
