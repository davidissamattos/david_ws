cmake_minimum_required(VERSION 2.8.3)
project(tb_self_experimentation)

if(CMAKE_COMPILER_IS_GNUCXX) 
	add_definitions(-std=gnu++0x) 
endif()

find_package(catkin REQUIRED COMPONENTS
  apriltags
  image_transport
  roscpp
  rospy
  sensor_msgs
  geometry_msgs
  visualization_msgs
  tf
  cv_bridge
  message_generation
  std_msgs
  cmake_modules
  nodelet
  move_base_msgs
  actionlib
  OpenCV
)

find_package(Eigen REQUIRED)

add_service_files(DIRECTORY srv
  FILES
  random_version.srv
  analyze_feedback.srv
)

add_message_files(DIRECTORY msg
  FILES
  object_loc.msg
  image_point.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  move_base_msgs
)

catkin_package(
  INCLUDE_DIRS
  LIBRARIES apriltag_detector 
  CATKIN_DEPENDS image_transport roscpp sensor_msgs geometry_msgs tf cv_bridge message_runtime std_msgs nodelet apriltags using_markers visualization_msgs move_base_msgs actionlib 
  DEPENDS Eigen OpenCV
)

###########
## Build ##
###########
include_directories(include  ${catkin_INCLUDE_DIRS})
include_directories(
  ${Eigen_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)


install(PROGRAMS pynodes/voice_feedback.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(approach_and_go src/approach_and_go.cpp)
target_link_libraries(approach_and_go ${catkin_LIBRARIES})
add_dependencies(approach_and_go approach_and_go ${${PROJECT_NAME}_EXPORTED_TARGETS})

add_executable(apriltag_detection src/apriltag_detection.cpp)
target_link_libraries(apriltag_detection ${catkin_LIBRARIES})
add_dependencies(apriltag_detection apriltag_detection ${${PROJECT_NAME}_EXPORTED_TARGETS})

add_executable(apriltag_filter src/apriltag_filter.cpp)
target_link_libraries(apriltag_filter ${catkin_LIBRARIES})
add_dependencies(apriltag_filter apriltag_filter ${${PROJECT_NAME}_EXPORTED_TARGETS})

add_executable(apriltag_tf_broadcaster src/apriltag_tf_broadcaster.cpp)
target_link_libraries(apriltag_tf_broadcaster ${catkin_LIBRARIES})
add_dependencies(apriltag_tf_broadcaster apriltag_tf_broadcaster ${${PROJECT_NAME}_EXPORTED_TARGETS})

add_executable(apriltag_tf_listener src/apriltag_tf_listener.cpp)
target_link_libraries(apriltag_tf_listener ${catkin_LIBRARIES})
add_dependencies(apriltag_tf_listener apriltag_tf_listener ${${PROJECT_NAME}_EXPORTED_TARGETS})

add_executable(tb_tf_listener src/tb_tf_listener.cpp)
target_link_libraries(tb_tf_listener ${catkin_LIBRARIES})
add_dependencies(tb_tf_listener tb_tf_listener ${${PROJECT_NAME}_EXPORTED_TARGETS})

add_executable(apriltag_rviz_markers src/apriltag_rviz_markers.cpp)
target_link_libraries(apriltag_rviz_markers ${catkin_LIBRARIES})
add_dependencies(apriltag_rviz_markers apriltag_rviz_markers ${${PROJECT_NAME}_EXPORTED_TARGETS})

add_executable(goal_rviz_markers src/goal_rviz_markers.cpp)
target_link_libraries(goal_rviz_markers ${catkin_LIBRARIES})
add_dependencies(goal_rviz_markers goal_rviz_markers ${${PROJECT_NAME}_EXPORTED_TARGETS})

add_executable(awaypoint_rviz_markers src/awaypoint_rviz_markers.cpp)
target_link_libraries(awaypoint_rviz_markers ${catkin_LIBRARIES})
add_dependencies(awaypoint_rviz_markers goal_rviz_markers ${${PROJECT_NAME}_EXPORTED_TARGETS})

add_executable(robot_feedback src/robot_feedback.cpp)
target_link_libraries(robot_feedback ${catkin_LIBRARIES})
add_dependencies(robot_feedback robot_feedback ${${PROJECT_NAME}_EXPORTED_TARGETS})
