# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "ptu_control: 0 messages, 1 services")

set(MSG_I_FLAGS "-Isensor_msgs:/opt/ros/noetic/share/sensor_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/noetic/share/geometry_msgs/cmake/../msg;-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(ptu_control_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/john/Projects/PTU-Control/src/ptu_control/srv/pan_tilt.srv" NAME_WE)
add_custom_target(_ptu_control_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "ptu_control" "/home/john/Projects/PTU-Control/src/ptu_control/srv/pan_tilt.srv" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages

### Generating Services
_generate_srv_cpp(ptu_control
  "/home/john/Projects/PTU-Control/src/ptu_control/srv/pan_tilt.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/ptu_control
)

### Generating Module File
_generate_module_cpp(ptu_control
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/ptu_control
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(ptu_control_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(ptu_control_generate_messages ptu_control_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/john/Projects/PTU-Control/src/ptu_control/srv/pan_tilt.srv" NAME_WE)
add_dependencies(ptu_control_generate_messages_cpp _ptu_control_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(ptu_control_gencpp)
add_dependencies(ptu_control_gencpp ptu_control_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS ptu_control_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages

### Generating Services
_generate_srv_eus(ptu_control
  "/home/john/Projects/PTU-Control/src/ptu_control/srv/pan_tilt.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/ptu_control
)

### Generating Module File
_generate_module_eus(ptu_control
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/ptu_control
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(ptu_control_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(ptu_control_generate_messages ptu_control_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/john/Projects/PTU-Control/src/ptu_control/srv/pan_tilt.srv" NAME_WE)
add_dependencies(ptu_control_generate_messages_eus _ptu_control_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(ptu_control_geneus)
add_dependencies(ptu_control_geneus ptu_control_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS ptu_control_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages

### Generating Services
_generate_srv_lisp(ptu_control
  "/home/john/Projects/PTU-Control/src/ptu_control/srv/pan_tilt.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/ptu_control
)

### Generating Module File
_generate_module_lisp(ptu_control
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/ptu_control
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(ptu_control_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(ptu_control_generate_messages ptu_control_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/john/Projects/PTU-Control/src/ptu_control/srv/pan_tilt.srv" NAME_WE)
add_dependencies(ptu_control_generate_messages_lisp _ptu_control_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(ptu_control_genlisp)
add_dependencies(ptu_control_genlisp ptu_control_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS ptu_control_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages

### Generating Services
_generate_srv_nodejs(ptu_control
  "/home/john/Projects/PTU-Control/src/ptu_control/srv/pan_tilt.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/ptu_control
)

### Generating Module File
_generate_module_nodejs(ptu_control
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/ptu_control
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(ptu_control_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(ptu_control_generate_messages ptu_control_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/john/Projects/PTU-Control/src/ptu_control/srv/pan_tilt.srv" NAME_WE)
add_dependencies(ptu_control_generate_messages_nodejs _ptu_control_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(ptu_control_gennodejs)
add_dependencies(ptu_control_gennodejs ptu_control_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS ptu_control_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages

### Generating Services
_generate_srv_py(ptu_control
  "/home/john/Projects/PTU-Control/src/ptu_control/srv/pan_tilt.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ptu_control
)

### Generating Module File
_generate_module_py(ptu_control
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ptu_control
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(ptu_control_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(ptu_control_generate_messages ptu_control_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/john/Projects/PTU-Control/src/ptu_control/srv/pan_tilt.srv" NAME_WE)
add_dependencies(ptu_control_generate_messages_py _ptu_control_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(ptu_control_genpy)
add_dependencies(ptu_control_genpy ptu_control_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS ptu_control_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/ptu_control)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/ptu_control
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET sensor_msgs_generate_messages_cpp)
  add_dependencies(ptu_control_generate_messages_cpp sensor_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/ptu_control)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/ptu_control
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET sensor_msgs_generate_messages_eus)
  add_dependencies(ptu_control_generate_messages_eus sensor_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/ptu_control)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/ptu_control
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET sensor_msgs_generate_messages_lisp)
  add_dependencies(ptu_control_generate_messages_lisp sensor_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/ptu_control)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/ptu_control
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET sensor_msgs_generate_messages_nodejs)
  add_dependencies(ptu_control_generate_messages_nodejs sensor_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ptu_control)
  install(CODE "execute_process(COMMAND \"/usr/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ptu_control\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ptu_control
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET sensor_msgs_generate_messages_py)
  add_dependencies(ptu_control_generate_messages_py sensor_msgs_generate_messages_py)
endif()
