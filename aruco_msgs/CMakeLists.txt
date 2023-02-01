cmake_minimum_required(VERSION 2.8.3)
project(aruco_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  geometry_msgs
  sensor_msgs
  std_msgs)

add_message_files( FILES Marker.msg MarkerArray.msg )
add_service_files(
  FILES
  MarkerReq.srv
)
generate_messages( DEPENDENCIES std_msgs geometry_msgs sensor_msgs)

## Generate services in the 'srv' folder


catkin_package(
  CATKIN_DEPENDS
    message_runtime
    geometry_msgs
    sensor_msgs
    std_msgs
)
