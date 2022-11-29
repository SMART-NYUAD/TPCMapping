# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "image_exposure_msgs: 3 messages, 0 services")

set(MSG_I_FLAGS "-Iimage_exposure_msgs:/home/john/Projects/PTU-Control/src/image_exposure_msgs/msg;-Istatistics_msgs:/home/john/Projects/PTU-Control/src/statistics_msgs/msg;-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(image_exposure_msgs_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/john/Projects/PTU-Control/src/image_exposure_msgs/msg/ExposureSequence.msg" NAME_WE)
add_custom_target(_image_exposure_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "image_exposure_msgs" "/home/john/Projects/PTU-Control/src/image_exposure_msgs/msg/ExposureSequence.msg" "std_msgs/Header"
)

get_filename_component(_filename "/home/john/Projects/PTU-Control/src/image_exposure_msgs/msg/ImageExposureStatistics.msg" NAME_WE)
add_custom_target(_image_exposure_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "image_exposure_msgs" "/home/john/Projects/PTU-Control/src/image_exposure_msgs/msg/ImageExposureStatistics.msg" "statistics_msgs/Stats1D"
)

get_filename_component(_filename "/home/john/Projects/PTU-Control/src/image_exposure_msgs/msg/SequenceExposureStatistics.msg" NAME_WE)
add_custom_target(_image_exposure_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "image_exposure_msgs" "/home/john/Projects/PTU-Control/src/image_exposure_msgs/msg/SequenceExposureStatistics.msg" "image_exposure_msgs/ImageExposureStatistics:statistics_msgs/Stats1D"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(image_exposure_msgs
  "/home/john/Projects/PTU-Control/src/image_exposure_msgs/msg/ExposureSequence.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/image_exposure_msgs
)
_generate_msg_cpp(image_exposure_msgs
  "/home/john/Projects/PTU-Control/src/image_exposure_msgs/msg/ImageExposureStatistics.msg"
  "${MSG_I_FLAGS}"
  "/home/john/Projects/PTU-Control/src/statistics_msgs/msg/Stats1D.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/image_exposure_msgs
)
_generate_msg_cpp(image_exposure_msgs
  "/home/john/Projects/PTU-Control/src/image_exposure_msgs/msg/SequenceExposureStatistics.msg"
  "${MSG_I_FLAGS}"
  "/home/john/Projects/PTU-Control/src/image_exposure_msgs/msg/ImageExposureStatistics.msg;/home/john/Projects/PTU-Control/src/statistics_msgs/msg/Stats1D.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/image_exposure_msgs
)

### Generating Services

### Generating Module File
_generate_module_cpp(image_exposure_msgs
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/image_exposure_msgs
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(image_exposure_msgs_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(image_exposure_msgs_generate_messages image_exposure_msgs_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/john/Projects/PTU-Control/src/image_exposure_msgs/msg/ExposureSequence.msg" NAME_WE)
add_dependencies(image_exposure_msgs_generate_messages_cpp _image_exposure_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/john/Projects/PTU-Control/src/image_exposure_msgs/msg/ImageExposureStatistics.msg" NAME_WE)
add_dependencies(image_exposure_msgs_generate_messages_cpp _image_exposure_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/john/Projects/PTU-Control/src/image_exposure_msgs/msg/SequenceExposureStatistics.msg" NAME_WE)
add_dependencies(image_exposure_msgs_generate_messages_cpp _image_exposure_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(image_exposure_msgs_gencpp)
add_dependencies(image_exposure_msgs_gencpp image_exposure_msgs_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS image_exposure_msgs_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(image_exposure_msgs
  "/home/john/Projects/PTU-Control/src/image_exposure_msgs/msg/ExposureSequence.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/image_exposure_msgs
)
_generate_msg_eus(image_exposure_msgs
  "/home/john/Projects/PTU-Control/src/image_exposure_msgs/msg/ImageExposureStatistics.msg"
  "${MSG_I_FLAGS}"
  "/home/john/Projects/PTU-Control/src/statistics_msgs/msg/Stats1D.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/image_exposure_msgs
)
_generate_msg_eus(image_exposure_msgs
  "/home/john/Projects/PTU-Control/src/image_exposure_msgs/msg/SequenceExposureStatistics.msg"
  "${MSG_I_FLAGS}"
  "/home/john/Projects/PTU-Control/src/image_exposure_msgs/msg/ImageExposureStatistics.msg;/home/john/Projects/PTU-Control/src/statistics_msgs/msg/Stats1D.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/image_exposure_msgs
)

### Generating Services

### Generating Module File
_generate_module_eus(image_exposure_msgs
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/image_exposure_msgs
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(image_exposure_msgs_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(image_exposure_msgs_generate_messages image_exposure_msgs_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/john/Projects/PTU-Control/src/image_exposure_msgs/msg/ExposureSequence.msg" NAME_WE)
add_dependencies(image_exposure_msgs_generate_messages_eus _image_exposure_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/john/Projects/PTU-Control/src/image_exposure_msgs/msg/ImageExposureStatistics.msg" NAME_WE)
add_dependencies(image_exposure_msgs_generate_messages_eus _image_exposure_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/john/Projects/PTU-Control/src/image_exposure_msgs/msg/SequenceExposureStatistics.msg" NAME_WE)
add_dependencies(image_exposure_msgs_generate_messages_eus _image_exposure_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(image_exposure_msgs_geneus)
add_dependencies(image_exposure_msgs_geneus image_exposure_msgs_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS image_exposure_msgs_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(image_exposure_msgs
  "/home/john/Projects/PTU-Control/src/image_exposure_msgs/msg/ExposureSequence.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/image_exposure_msgs
)
_generate_msg_lisp(image_exposure_msgs
  "/home/john/Projects/PTU-Control/src/image_exposure_msgs/msg/ImageExposureStatistics.msg"
  "${MSG_I_FLAGS}"
  "/home/john/Projects/PTU-Control/src/statistics_msgs/msg/Stats1D.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/image_exposure_msgs
)
_generate_msg_lisp(image_exposure_msgs
  "/home/john/Projects/PTU-Control/src/image_exposure_msgs/msg/SequenceExposureStatistics.msg"
  "${MSG_I_FLAGS}"
  "/home/john/Projects/PTU-Control/src/image_exposure_msgs/msg/ImageExposureStatistics.msg;/home/john/Projects/PTU-Control/src/statistics_msgs/msg/Stats1D.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/image_exposure_msgs
)

### Generating Services

### Generating Module File
_generate_module_lisp(image_exposure_msgs
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/image_exposure_msgs
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(image_exposure_msgs_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(image_exposure_msgs_generate_messages image_exposure_msgs_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/john/Projects/PTU-Control/src/image_exposure_msgs/msg/ExposureSequence.msg" NAME_WE)
add_dependencies(image_exposure_msgs_generate_messages_lisp _image_exposure_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/john/Projects/PTU-Control/src/image_exposure_msgs/msg/ImageExposureStatistics.msg" NAME_WE)
add_dependencies(image_exposure_msgs_generate_messages_lisp _image_exposure_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/john/Projects/PTU-Control/src/image_exposure_msgs/msg/SequenceExposureStatistics.msg" NAME_WE)
add_dependencies(image_exposure_msgs_generate_messages_lisp _image_exposure_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(image_exposure_msgs_genlisp)
add_dependencies(image_exposure_msgs_genlisp image_exposure_msgs_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS image_exposure_msgs_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(image_exposure_msgs
  "/home/john/Projects/PTU-Control/src/image_exposure_msgs/msg/ExposureSequence.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/image_exposure_msgs
)
_generate_msg_nodejs(image_exposure_msgs
  "/home/john/Projects/PTU-Control/src/image_exposure_msgs/msg/ImageExposureStatistics.msg"
  "${MSG_I_FLAGS}"
  "/home/john/Projects/PTU-Control/src/statistics_msgs/msg/Stats1D.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/image_exposure_msgs
)
_generate_msg_nodejs(image_exposure_msgs
  "/home/john/Projects/PTU-Control/src/image_exposure_msgs/msg/SequenceExposureStatistics.msg"
  "${MSG_I_FLAGS}"
  "/home/john/Projects/PTU-Control/src/image_exposure_msgs/msg/ImageExposureStatistics.msg;/home/john/Projects/PTU-Control/src/statistics_msgs/msg/Stats1D.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/image_exposure_msgs
)

### Generating Services

### Generating Module File
_generate_module_nodejs(image_exposure_msgs
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/image_exposure_msgs
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(image_exposure_msgs_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(image_exposure_msgs_generate_messages image_exposure_msgs_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/john/Projects/PTU-Control/src/image_exposure_msgs/msg/ExposureSequence.msg" NAME_WE)
add_dependencies(image_exposure_msgs_generate_messages_nodejs _image_exposure_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/john/Projects/PTU-Control/src/image_exposure_msgs/msg/ImageExposureStatistics.msg" NAME_WE)
add_dependencies(image_exposure_msgs_generate_messages_nodejs _image_exposure_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/john/Projects/PTU-Control/src/image_exposure_msgs/msg/SequenceExposureStatistics.msg" NAME_WE)
add_dependencies(image_exposure_msgs_generate_messages_nodejs _image_exposure_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(image_exposure_msgs_gennodejs)
add_dependencies(image_exposure_msgs_gennodejs image_exposure_msgs_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS image_exposure_msgs_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(image_exposure_msgs
  "/home/john/Projects/PTU-Control/src/image_exposure_msgs/msg/ExposureSequence.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/image_exposure_msgs
)
_generate_msg_py(image_exposure_msgs
  "/home/john/Projects/PTU-Control/src/image_exposure_msgs/msg/ImageExposureStatistics.msg"
  "${MSG_I_FLAGS}"
  "/home/john/Projects/PTU-Control/src/statistics_msgs/msg/Stats1D.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/image_exposure_msgs
)
_generate_msg_py(image_exposure_msgs
  "/home/john/Projects/PTU-Control/src/image_exposure_msgs/msg/SequenceExposureStatistics.msg"
  "${MSG_I_FLAGS}"
  "/home/john/Projects/PTU-Control/src/image_exposure_msgs/msg/ImageExposureStatistics.msg;/home/john/Projects/PTU-Control/src/statistics_msgs/msg/Stats1D.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/image_exposure_msgs
)

### Generating Services

### Generating Module File
_generate_module_py(image_exposure_msgs
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/image_exposure_msgs
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(image_exposure_msgs_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(image_exposure_msgs_generate_messages image_exposure_msgs_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/john/Projects/PTU-Control/src/image_exposure_msgs/msg/ExposureSequence.msg" NAME_WE)
add_dependencies(image_exposure_msgs_generate_messages_py _image_exposure_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/john/Projects/PTU-Control/src/image_exposure_msgs/msg/ImageExposureStatistics.msg" NAME_WE)
add_dependencies(image_exposure_msgs_generate_messages_py _image_exposure_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/john/Projects/PTU-Control/src/image_exposure_msgs/msg/SequenceExposureStatistics.msg" NAME_WE)
add_dependencies(image_exposure_msgs_generate_messages_py _image_exposure_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(image_exposure_msgs_genpy)
add_dependencies(image_exposure_msgs_genpy image_exposure_msgs_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS image_exposure_msgs_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/image_exposure_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/image_exposure_msgs
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET statistics_msgs_generate_messages_cpp)
  add_dependencies(image_exposure_msgs_generate_messages_cpp statistics_msgs_generate_messages_cpp)
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(image_exposure_msgs_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/image_exposure_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/image_exposure_msgs
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET statistics_msgs_generate_messages_eus)
  add_dependencies(image_exposure_msgs_generate_messages_eus statistics_msgs_generate_messages_eus)
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(image_exposure_msgs_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/image_exposure_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/image_exposure_msgs
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET statistics_msgs_generate_messages_lisp)
  add_dependencies(image_exposure_msgs_generate_messages_lisp statistics_msgs_generate_messages_lisp)
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(image_exposure_msgs_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/image_exposure_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/image_exposure_msgs
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET statistics_msgs_generate_messages_nodejs)
  add_dependencies(image_exposure_msgs_generate_messages_nodejs statistics_msgs_generate_messages_nodejs)
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(image_exposure_msgs_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/image_exposure_msgs)
  install(CODE "execute_process(COMMAND \"/usr/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/image_exposure_msgs\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/image_exposure_msgs
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET statistics_msgs_generate_messages_py)
  add_dependencies(image_exposure_msgs_generate_messages_py statistics_msgs_generate_messages_py)
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(image_exposure_msgs_generate_messages_py std_msgs_generate_messages_py)
endif()
