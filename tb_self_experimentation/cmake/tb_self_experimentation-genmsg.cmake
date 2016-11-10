# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "tb_self_experimentation: 2 messages, 0 services")

set(MSG_I_FLAGS "-Itb_self_experimentation:/home/davidis/david_ws/src/tb_self_experimentation/msg;-Istd_msgs:/opt/ros/indigo/share/std_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/indigo/share/geometry_msgs/cmake/../msg;-Imove_base_msgs:/opt/ros/indigo/share/move_base_msgs/cmake/../msg;-Iactionlib_msgs:/opt/ros/indigo/share/actionlib_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(genlisp REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(tb_self_experimentation_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/davidis/david_ws/src/tb_self_experimentation/msg/image_point.msg" NAME_WE)
add_custom_target(_tb_self_experimentation_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "tb_self_experimentation" "/home/davidis/david_ws/src/tb_self_experimentation/msg/image_point.msg" "std_msgs/Int64:geometry_msgs/Point"
)

get_filename_component(_filename "/home/davidis/david_ws/src/tb_self_experimentation/msg/object_loc.msg" NAME_WE)
add_custom_target(_tb_self_experimentation_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "tb_self_experimentation" "/home/davidis/david_ws/src/tb_self_experimentation/msg/object_loc.msg" "geometry_msgs/Point"
)

#
#  langs = gencpp;genlisp;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(tb_self_experimentation
  "/home/davidis/david_ws/src/tb_self_experimentation/msg/image_point.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/std_msgs/cmake/../msg/Int64.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/tb_self_experimentation
)
_generate_msg_cpp(tb_self_experimentation
  "/home/davidis/david_ws/src/tb_self_experimentation/msg/object_loc.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/tb_self_experimentation
)

### Generating Services

### Generating Module File
_generate_module_cpp(tb_self_experimentation
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/tb_self_experimentation
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(tb_self_experimentation_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(tb_self_experimentation_generate_messages tb_self_experimentation_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/davidis/david_ws/src/tb_self_experimentation/msg/image_point.msg" NAME_WE)
add_dependencies(tb_self_experimentation_generate_messages_cpp _tb_self_experimentation_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/davidis/david_ws/src/tb_self_experimentation/msg/object_loc.msg" NAME_WE)
add_dependencies(tb_self_experimentation_generate_messages_cpp _tb_self_experimentation_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(tb_self_experimentation_gencpp)
add_dependencies(tb_self_experimentation_gencpp tb_self_experimentation_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS tb_self_experimentation_generate_messages_cpp)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(tb_self_experimentation
  "/home/davidis/david_ws/src/tb_self_experimentation/msg/image_point.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/std_msgs/cmake/../msg/Int64.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/tb_self_experimentation
)
_generate_msg_lisp(tb_self_experimentation
  "/home/davidis/david_ws/src/tb_self_experimentation/msg/object_loc.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/tb_self_experimentation
)

### Generating Services

### Generating Module File
_generate_module_lisp(tb_self_experimentation
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/tb_self_experimentation
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(tb_self_experimentation_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(tb_self_experimentation_generate_messages tb_self_experimentation_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/davidis/david_ws/src/tb_self_experimentation/msg/image_point.msg" NAME_WE)
add_dependencies(tb_self_experimentation_generate_messages_lisp _tb_self_experimentation_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/davidis/david_ws/src/tb_self_experimentation/msg/object_loc.msg" NAME_WE)
add_dependencies(tb_self_experimentation_generate_messages_lisp _tb_self_experimentation_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(tb_self_experimentation_genlisp)
add_dependencies(tb_self_experimentation_genlisp tb_self_experimentation_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS tb_self_experimentation_generate_messages_lisp)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(tb_self_experimentation
  "/home/davidis/david_ws/src/tb_self_experimentation/msg/image_point.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/std_msgs/cmake/../msg/Int64.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/tb_self_experimentation
)
_generate_msg_py(tb_self_experimentation
  "/home/davidis/david_ws/src/tb_self_experimentation/msg/object_loc.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/tb_self_experimentation
)

### Generating Services

### Generating Module File
_generate_module_py(tb_self_experimentation
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/tb_self_experimentation
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(tb_self_experimentation_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(tb_self_experimentation_generate_messages tb_self_experimentation_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/davidis/david_ws/src/tb_self_experimentation/msg/image_point.msg" NAME_WE)
add_dependencies(tb_self_experimentation_generate_messages_py _tb_self_experimentation_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/davidis/david_ws/src/tb_self_experimentation/msg/object_loc.msg" NAME_WE)
add_dependencies(tb_self_experimentation_generate_messages_py _tb_self_experimentation_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(tb_self_experimentation_genpy)
add_dependencies(tb_self_experimentation_genpy tb_self_experimentation_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS tb_self_experimentation_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/tb_self_experimentation)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/tb_self_experimentation
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
add_dependencies(tb_self_experimentation_generate_messages_cpp std_msgs_generate_messages_cpp)
add_dependencies(tb_self_experimentation_generate_messages_cpp geometry_msgs_generate_messages_cpp)
add_dependencies(tb_self_experimentation_generate_messages_cpp move_base_msgs_generate_messages_cpp)

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/tb_self_experimentation)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/tb_self_experimentation
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
add_dependencies(tb_self_experimentation_generate_messages_lisp std_msgs_generate_messages_lisp)
add_dependencies(tb_self_experimentation_generate_messages_lisp geometry_msgs_generate_messages_lisp)
add_dependencies(tb_self_experimentation_generate_messages_lisp move_base_msgs_generate_messages_lisp)

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/tb_self_experimentation)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/tb_self_experimentation\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/tb_self_experimentation
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
add_dependencies(tb_self_experimentation_generate_messages_py std_msgs_generate_messages_py)
add_dependencies(tb_self_experimentation_generate_messages_py geometry_msgs_generate_messages_py)
add_dependencies(tb_self_experimentation_generate_messages_py move_base_msgs_generate_messages_py)
