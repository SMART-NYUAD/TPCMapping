# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "wfov_camera_msgs: 3 messages, 0 services")

set(MSG_I_FLAGS "-Iwfov_camera_msgs:/home/john/Projects/PTU-Control/src/wfov_camera_msgs/msg;-Isensor_msgs:/opt/ros/noetic/share/sensor_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/noetic/share/geometry_msgs/cmake/../msg;-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(wfov_camera_msgs_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/john/Projects/PTU-Control/src/wfov_camera_msgs/msg/WFOVCompressedImage.msg" NAME_WE)
add_custom_target(_wfov_camera_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "wfov_camera_msgs" "/home/john/Projects/PTU-Control/src/wfov_camera_msgs/msg/WFOVCompressedImage.msg" "std_msgs/Header:geometry_msgs/Quaternion:geometry_msgs/Vector3:geometry_msgs/Transform:sensor_msgs/CompressedImage:sensor_msgs/RegionOfInterest:geometry_msgs/TransformStamped:sensor_msgs/CameraInfo"
)

get_filename_component(_filename "/home/john/Projects/PTU-Control/src/wfov_camera_msgs/msg/WFOVImage.msg" NAME_WE)
add_custom_target(_wfov_camera_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "wfov_camera_msgs" "/home/john/Projects/PTU-Control/src/wfov_camera_msgs/msg/WFOVImage.msg" "sensor_msgs/RegionOfInterest:std_msgs/Header:sensor_msgs/CameraInfo:sensor_msgs/Image"
)

get_filename_component(_filename "/home/john/Projects/PTU-Control/src/wfov_camera_msgs/msg/WFOVTrigger.msg" NAME_WE)
add_custom_target(_wfov_camera_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "wfov_camera_msgs" "/home/john/Projects/PTU-Control/src/wfov_camera_msgs/msg/WFOVTrigger.msg" "std_msgs/Header"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(wfov_camera_msgs
  "/home/john/Projects/PTU-Control/src/wfov_camera_msgs/msg/WFOVCompressedImage.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Transform.msg;/opt/ros/noetic/share/sensor_msgs/cmake/../msg/CompressedImage.msg;/opt/ros/noetic/share/sensor_msgs/cmake/../msg/RegionOfInterest.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/TransformStamped.msg;/opt/ros/noetic/share/sensor_msgs/cmake/../msg/CameraInfo.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/wfov_camera_msgs
)
_generate_msg_cpp(wfov_camera_msgs
  "/home/john/Projects/PTU-Control/src/wfov_camera_msgs/msg/WFOVImage.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/sensor_msgs/cmake/../msg/RegionOfInterest.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/sensor_msgs/cmake/../msg/CameraInfo.msg;/opt/ros/noetic/share/sensor_msgs/cmake/../msg/Image.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/wfov_camera_msgs
)
_generate_msg_cpp(wfov_camera_msgs
  "/home/john/Projects/PTU-Control/src/wfov_camera_msgs/msg/WFOVTrigger.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/wfov_camera_msgs
)

### Generating Services

### Generating Module File
_generate_module_cpp(wfov_camera_msgs
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/wfov_camera_msgs
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(wfov_camera_msgs_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(wfov_camera_msgs_generate_messages wfov_camera_msgs_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/john/Projects/PTU-Control/src/wfov_camera_msgs/msg/WFOVCompressedImage.msg" NAME_WE)
add_dependencies(wfov_camera_msgs_generate_messages_cpp _wfov_camera_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/john/Projects/PTU-Control/src/wfov_camera_msgs/msg/WFOVImage.msg" NAME_WE)
add_dependencies(wfov_camera_msgs_generate_messages_cpp _wfov_camera_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/john/Projects/PTU-Control/src/wfov_camera_msgs/msg/WFOVTrigger.msg" NAME_WE)
add_dependencies(wfov_camera_msgs_generate_messages_cpp _wfov_camera_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(wfov_camera_msgs_gencpp)
add_dependencies(wfov_camera_msgs_gencpp wfov_camera_msgs_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS wfov_camera_msgs_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(wfov_camera_msgs
  "/home/john/Projects/PTU-Control/src/wfov_camera_msgs/msg/WFOVCompressedImage.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Transform.msg;/opt/ros/noetic/share/sensor_msgs/cmake/../msg/CompressedImage.msg;/opt/ros/noetic/share/sensor_msgs/cmake/../msg/RegionOfInterest.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/TransformStamped.msg;/opt/ros/noetic/share/sensor_msgs/cmake/../msg/CameraInfo.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/wfov_camera_msgs
)
_generate_msg_eus(wfov_camera_msgs
  "/home/john/Projects/PTU-Control/src/wfov_camera_msgs/msg/WFOVImage.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/sensor_msgs/cmake/../msg/RegionOfInterest.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/sensor_msgs/cmake/../msg/CameraInfo.msg;/opt/ros/noetic/share/sensor_msgs/cmake/../msg/Image.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/wfov_camera_msgs
)
_generate_msg_eus(wfov_camera_msgs
  "/home/john/Projects/PTU-Control/src/wfov_camera_msgs/msg/WFOVTrigger.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/wfov_camera_msgs
)

### Generating Services

### Generating Module File
_generate_module_eus(wfov_camera_msgs
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/wfov_camera_msgs
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(wfov_camera_msgs_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(wfov_camera_msgs_generate_messages wfov_camera_msgs_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/john/Projects/PTU-Control/src/wfov_camera_msgs/msg/WFOVCompressedImage.msg" NAME_WE)
add_dependencies(wfov_camera_msgs_generate_messages_eus _wfov_camera_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/john/Projects/PTU-Control/src/wfov_camera_msgs/msg/WFOVImage.msg" NAME_WE)
add_dependencies(wfov_camera_msgs_generate_messages_eus _wfov_camera_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/john/Projects/PTU-Control/src/wfov_camera_msgs/msg/WFOVTrigger.msg" NAME_WE)
add_dependencies(wfov_camera_msgs_generate_messages_eus _wfov_camera_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(wfov_camera_msgs_geneus)
add_dependencies(wfov_camera_msgs_geneus wfov_camera_msgs_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS wfov_camera_msgs_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(wfov_camera_msgs
  "/home/john/Projects/PTU-Control/src/wfov_camera_msgs/msg/WFOVCompressedImage.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Transform.msg;/opt/ros/noetic/share/sensor_msgs/cmake/../msg/CompressedImage.msg;/opt/ros/noetic/share/sensor_msgs/cmake/../msg/RegionOfInterest.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/TransformStamped.msg;/opt/ros/noetic/share/sensor_msgs/cmake/../msg/CameraInfo.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/wfov_camera_msgs
)
_generate_msg_lisp(wfov_camera_msgs
  "/home/john/Projects/PTU-Control/src/wfov_camera_msgs/msg/WFOVImage.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/sensor_msgs/cmake/../msg/RegionOfInterest.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/sensor_msgs/cmake/../msg/CameraInfo.msg;/opt/ros/noetic/share/sensor_msgs/cmake/../msg/Image.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/wfov_camera_msgs
)
_generate_msg_lisp(wfov_camera_msgs
  "/home/john/Projects/PTU-Control/src/wfov_camera_msgs/msg/WFOVTrigger.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/wfov_camera_msgs
)

### Generating Services

### Generating Module File
_generate_module_lisp(wfov_camera_msgs
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/wfov_camera_msgs
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(wfov_camera_msgs_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(wfov_camera_msgs_generate_messages wfov_camera_msgs_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/john/Projects/PTU-Control/src/wfov_camera_msgs/msg/WFOVCompressedImage.msg" NAME_WE)
add_dependencies(wfov_camera_msgs_generate_messages_lisp _wfov_camera_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/john/Projects/PTU-Control/src/wfov_camera_msgs/msg/WFOVImage.msg" NAME_WE)
add_dependencies(wfov_camera_msgs_generate_messages_lisp _wfov_camera_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/john/Projects/PTU-Control/src/wfov_camera_msgs/msg/WFOVTrigger.msg" NAME_WE)
add_dependencies(wfov_camera_msgs_generate_messages_lisp _wfov_camera_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(wfov_camera_msgs_genlisp)
add_dependencies(wfov_camera_msgs_genlisp wfov_camera_msgs_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS wfov_camera_msgs_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(wfov_camera_msgs
  "/home/john/Projects/PTU-Control/src/wfov_camera_msgs/msg/WFOVCompressedImage.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Transform.msg;/opt/ros/noetic/share/sensor_msgs/cmake/../msg/CompressedImage.msg;/opt/ros/noetic/share/sensor_msgs/cmake/../msg/RegionOfInterest.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/TransformStamped.msg;/opt/ros/noetic/share/sensor_msgs/cmake/../msg/CameraInfo.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/wfov_camera_msgs
)
_generate_msg_nodejs(wfov_camera_msgs
  "/home/john/Projects/PTU-Control/src/wfov_camera_msgs/msg/WFOVImage.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/sensor_msgs/cmake/../msg/RegionOfInterest.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/sensor_msgs/cmake/../msg/CameraInfo.msg;/opt/ros/noetic/share/sensor_msgs/cmake/../msg/Image.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/wfov_camera_msgs
)
_generate_msg_nodejs(wfov_camera_msgs
  "/home/john/Projects/PTU-Control/src/wfov_camera_msgs/msg/WFOVTrigger.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/wfov_camera_msgs
)

### Generating Services

### Generating Module File
_generate_module_nodejs(wfov_camera_msgs
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/wfov_camera_msgs
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(wfov_camera_msgs_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(wfov_camera_msgs_generate_messages wfov_camera_msgs_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/john/Projects/PTU-Control/src/wfov_camera_msgs/msg/WFOVCompressedImage.msg" NAME_WE)
add_dependencies(wfov_camera_msgs_generate_messages_nodejs _wfov_camera_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/john/Projects/PTU-Control/src/wfov_camera_msgs/msg/WFOVImage.msg" NAME_WE)
add_dependencies(wfov_camera_msgs_generate_messages_nodejs _wfov_camera_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/john/Projects/PTU-Control/src/wfov_camera_msgs/msg/WFOVTrigger.msg" NAME_WE)
add_dependencies(wfov_camera_msgs_generate_messages_nodejs _wfov_camera_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(wfov_camera_msgs_gennodejs)
add_dependencies(wfov_camera_msgs_gennodejs wfov_camera_msgs_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS wfov_camera_msgs_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(wfov_camera_msgs
  "/home/john/Projects/PTU-Control/src/wfov_camera_msgs/msg/WFOVCompressedImage.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Transform.msg;/opt/ros/noetic/share/sensor_msgs/cmake/../msg/CompressedImage.msg;/opt/ros/noetic/share/sensor_msgs/cmake/../msg/RegionOfInterest.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/TransformStamped.msg;/opt/ros/noetic/share/sensor_msgs/cmake/../msg/CameraInfo.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/wfov_camera_msgs
)
_generate_msg_py(wfov_camera_msgs
  "/home/john/Projects/PTU-Control/src/wfov_camera_msgs/msg/WFOVImage.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/sensor_msgs/cmake/../msg/RegionOfInterest.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/sensor_msgs/cmake/../msg/CameraInfo.msg;/opt/ros/noetic/share/sensor_msgs/cmake/../msg/Image.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/wfov_camera_msgs
)
_generate_msg_py(wfov_camera_msgs
  "/home/john/Projects/PTU-Control/src/wfov_camera_msgs/msg/WFOVTrigger.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/wfov_camera_msgs
)

### Generating Services

### Generating Module File
_generate_module_py(wfov_camera_msgs
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/wfov_camera_msgs
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(wfov_camera_msgs_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(wfov_camera_msgs_generate_messages wfov_camera_msgs_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/john/Projects/PTU-Control/src/wfov_camera_msgs/msg/WFOVCompressedImage.msg" NAME_WE)
add_dependencies(wfov_camera_msgs_generate_messages_py _wfov_camera_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/john/Projects/PTU-Control/src/wfov_camera_msgs/msg/WFOVImage.msg" NAME_WE)
add_dependencies(wfov_camera_msgs_generate_messages_py _wfov_camera_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/john/Projects/PTU-Control/src/wfov_camera_msgs/msg/WFOVTrigger.msg" NAME_WE)
add_dependencies(wfov_camera_msgs_generate_messages_py _wfov_camera_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(wfov_camera_msgs_genpy)
add_dependencies(wfov_camera_msgs_genpy wfov_camera_msgs_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS wfov_camera_msgs_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/wfov_camera_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/wfov_camera_msgs
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET sensor_msgs_generate_messages_cpp)
  add_dependencies(wfov_camera_msgs_generate_messages_cpp sensor_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/wfov_camera_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/wfov_camera_msgs
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET sensor_msgs_generate_messages_eus)
  add_dependencies(wfov_camera_msgs_generate_messages_eus sensor_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/wfov_camera_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/wfov_camera_msgs
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET sensor_msgs_generate_messages_lisp)
  add_dependencies(wfov_camera_msgs_generate_messages_lisp sensor_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/wfov_camera_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/wfov_camera_msgs
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET sensor_msgs_generate_messages_nodejs)
  add_dependencies(wfov_camera_msgs_generate_messages_nodejs sensor_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/wfov_camera_msgs)
  install(CODE "execute_process(COMMAND \"/usr/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/wfov_camera_msgs\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/wfov_camera_msgs
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET sensor_msgs_generate_messages_py)
  add_dependencies(wfov_camera_msgs_generate_messages_py sensor_msgs_generate_messages_py)
endif()
