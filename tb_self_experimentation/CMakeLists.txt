cmake_minimum_required(VERSION 2.8.3)
project(tb_self_experimentation)


find_package(catkin REQUIRED COMPONENTS
  apriltags
  image_transport
  roscpp
  rospy
  sensor_msgs
  geometry_msgs
  tf
  cv_bridge
  message_generation
  std_msgs
  cmake_modules
  nodelet
  wasp_custom_msgs
)

find_package(Eigen REQUIRED)

#add_message_files(
#  FILES
#)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  INCLUDE_DIRS
  LIBRARIES apriltag_detector
  CATKIN_DEPENDS image_transport roscpp sensor_msgs geometry_msgs tf cv_bridge message_runtime std_msgs nodelet apriltags wasp_custom_msgs
  DEPENDS Eigen
)

###########
## Build ##
###########
include_directories(include  ${catkin_INCLUDE_DIRS})
include_directories(
  ${Eigen_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)


add_executable(apriltag_detection src/apriltag_detection.cpp)
target_link_libraries(apriltag_detection ${catkin_LIBRARIES})
add_dependencies(apriltag_detection apriltag_detection)

add_executable(apriltag_filter src/apriltag_filter.cpp)
target_link_libraries(apriltag_filter ${catkin_LIBRARIES})
add_dependencies(apriltag_filter apriltag_filter)

add_executable(apriltag_tf_broadcaster src/apriltag_tf_broadcaster.cpp)
target_link_libraries(apriltag_tf_broadcaster ${catkin_LIBRARIES})
add_dependencies(apriltag_tf_broadcaster apriltag_tf_broadcaster)

