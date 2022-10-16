# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "ur3_driver: 3 messages, 0 services")

set(MSG_I_FLAGS "-Iur3_driver:/home/ur3/catkin_jushanc2/src/lab2andDriver/drivers/ur3_driver/msg;-Istd_msgs:/opt/ros/kinetic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(ur3_driver_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/ur3/catkin_jushanc2/src/lab2andDriver/drivers/ur3_driver/msg/position.msg" NAME_WE)
add_custom_target(_ur3_driver_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "ur3_driver" "/home/ur3/catkin_jushanc2/src/lab2andDriver/drivers/ur3_driver/msg/position.msg" ""
)

get_filename_component(_filename "/home/ur3/catkin_jushanc2/src/lab2andDriver/drivers/ur3_driver/msg/command.msg" NAME_WE)
add_custom_target(_ur3_driver_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "ur3_driver" "/home/ur3/catkin_jushanc2/src/lab2andDriver/drivers/ur3_driver/msg/command.msg" ""
)

get_filename_component(_filename "/home/ur3/catkin_jushanc2/src/lab2andDriver/drivers/ur3_driver/msg/gripper_input.msg" NAME_WE)
add_custom_target(_ur3_driver_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "ur3_driver" "/home/ur3/catkin_jushanc2/src/lab2andDriver/drivers/ur3_driver/msg/gripper_input.msg" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(ur3_driver
  "/home/ur3/catkin_jushanc2/src/lab2andDriver/drivers/ur3_driver/msg/position.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/ur3_driver
)
_generate_msg_cpp(ur3_driver
  "/home/ur3/catkin_jushanc2/src/lab2andDriver/drivers/ur3_driver/msg/command.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/ur3_driver
)
_generate_msg_cpp(ur3_driver
  "/home/ur3/catkin_jushanc2/src/lab2andDriver/drivers/ur3_driver/msg/gripper_input.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/ur3_driver
)

### Generating Services

### Generating Module File
_generate_module_cpp(ur3_driver
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/ur3_driver
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(ur3_driver_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(ur3_driver_generate_messages ur3_driver_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/ur3/catkin_jushanc2/src/lab2andDriver/drivers/ur3_driver/msg/position.msg" NAME_WE)
add_dependencies(ur3_driver_generate_messages_cpp _ur3_driver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ur3/catkin_jushanc2/src/lab2andDriver/drivers/ur3_driver/msg/command.msg" NAME_WE)
add_dependencies(ur3_driver_generate_messages_cpp _ur3_driver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ur3/catkin_jushanc2/src/lab2andDriver/drivers/ur3_driver/msg/gripper_input.msg" NAME_WE)
add_dependencies(ur3_driver_generate_messages_cpp _ur3_driver_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(ur3_driver_gencpp)
add_dependencies(ur3_driver_gencpp ur3_driver_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS ur3_driver_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(ur3_driver
  "/home/ur3/catkin_jushanc2/src/lab2andDriver/drivers/ur3_driver/msg/position.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/ur3_driver
)
_generate_msg_eus(ur3_driver
  "/home/ur3/catkin_jushanc2/src/lab2andDriver/drivers/ur3_driver/msg/command.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/ur3_driver
)
_generate_msg_eus(ur3_driver
  "/home/ur3/catkin_jushanc2/src/lab2andDriver/drivers/ur3_driver/msg/gripper_input.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/ur3_driver
)

### Generating Services

### Generating Module File
_generate_module_eus(ur3_driver
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/ur3_driver
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(ur3_driver_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(ur3_driver_generate_messages ur3_driver_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/ur3/catkin_jushanc2/src/lab2andDriver/drivers/ur3_driver/msg/position.msg" NAME_WE)
add_dependencies(ur3_driver_generate_messages_eus _ur3_driver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ur3/catkin_jushanc2/src/lab2andDriver/drivers/ur3_driver/msg/command.msg" NAME_WE)
add_dependencies(ur3_driver_generate_messages_eus _ur3_driver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ur3/catkin_jushanc2/src/lab2andDriver/drivers/ur3_driver/msg/gripper_input.msg" NAME_WE)
add_dependencies(ur3_driver_generate_messages_eus _ur3_driver_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(ur3_driver_geneus)
add_dependencies(ur3_driver_geneus ur3_driver_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS ur3_driver_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(ur3_driver
  "/home/ur3/catkin_jushanc2/src/lab2andDriver/drivers/ur3_driver/msg/position.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/ur3_driver
)
_generate_msg_lisp(ur3_driver
  "/home/ur3/catkin_jushanc2/src/lab2andDriver/drivers/ur3_driver/msg/command.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/ur3_driver
)
_generate_msg_lisp(ur3_driver
  "/home/ur3/catkin_jushanc2/src/lab2andDriver/drivers/ur3_driver/msg/gripper_input.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/ur3_driver
)

### Generating Services

### Generating Module File
_generate_module_lisp(ur3_driver
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/ur3_driver
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(ur3_driver_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(ur3_driver_generate_messages ur3_driver_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/ur3/catkin_jushanc2/src/lab2andDriver/drivers/ur3_driver/msg/position.msg" NAME_WE)
add_dependencies(ur3_driver_generate_messages_lisp _ur3_driver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ur3/catkin_jushanc2/src/lab2andDriver/drivers/ur3_driver/msg/command.msg" NAME_WE)
add_dependencies(ur3_driver_generate_messages_lisp _ur3_driver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ur3/catkin_jushanc2/src/lab2andDriver/drivers/ur3_driver/msg/gripper_input.msg" NAME_WE)
add_dependencies(ur3_driver_generate_messages_lisp _ur3_driver_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(ur3_driver_genlisp)
add_dependencies(ur3_driver_genlisp ur3_driver_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS ur3_driver_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(ur3_driver
  "/home/ur3/catkin_jushanc2/src/lab2andDriver/drivers/ur3_driver/msg/position.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/ur3_driver
)
_generate_msg_nodejs(ur3_driver
  "/home/ur3/catkin_jushanc2/src/lab2andDriver/drivers/ur3_driver/msg/command.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/ur3_driver
)
_generate_msg_nodejs(ur3_driver
  "/home/ur3/catkin_jushanc2/src/lab2andDriver/drivers/ur3_driver/msg/gripper_input.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/ur3_driver
)

### Generating Services

### Generating Module File
_generate_module_nodejs(ur3_driver
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/ur3_driver
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(ur3_driver_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(ur3_driver_generate_messages ur3_driver_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/ur3/catkin_jushanc2/src/lab2andDriver/drivers/ur3_driver/msg/position.msg" NAME_WE)
add_dependencies(ur3_driver_generate_messages_nodejs _ur3_driver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ur3/catkin_jushanc2/src/lab2andDriver/drivers/ur3_driver/msg/command.msg" NAME_WE)
add_dependencies(ur3_driver_generate_messages_nodejs _ur3_driver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ur3/catkin_jushanc2/src/lab2andDriver/drivers/ur3_driver/msg/gripper_input.msg" NAME_WE)
add_dependencies(ur3_driver_generate_messages_nodejs _ur3_driver_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(ur3_driver_gennodejs)
add_dependencies(ur3_driver_gennodejs ur3_driver_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS ur3_driver_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(ur3_driver
  "/home/ur3/catkin_jushanc2/src/lab2andDriver/drivers/ur3_driver/msg/position.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ur3_driver
)
_generate_msg_py(ur3_driver
  "/home/ur3/catkin_jushanc2/src/lab2andDriver/drivers/ur3_driver/msg/command.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ur3_driver
)
_generate_msg_py(ur3_driver
  "/home/ur3/catkin_jushanc2/src/lab2andDriver/drivers/ur3_driver/msg/gripper_input.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ur3_driver
)

### Generating Services

### Generating Module File
_generate_module_py(ur3_driver
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ur3_driver
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(ur3_driver_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(ur3_driver_generate_messages ur3_driver_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/ur3/catkin_jushanc2/src/lab2andDriver/drivers/ur3_driver/msg/position.msg" NAME_WE)
add_dependencies(ur3_driver_generate_messages_py _ur3_driver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ur3/catkin_jushanc2/src/lab2andDriver/drivers/ur3_driver/msg/command.msg" NAME_WE)
add_dependencies(ur3_driver_generate_messages_py _ur3_driver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ur3/catkin_jushanc2/src/lab2andDriver/drivers/ur3_driver/msg/gripper_input.msg" NAME_WE)
add_dependencies(ur3_driver_generate_messages_py _ur3_driver_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(ur3_driver_genpy)
add_dependencies(ur3_driver_genpy ur3_driver_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS ur3_driver_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/ur3_driver)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/ur3_driver
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(ur3_driver_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/ur3_driver)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/ur3_driver
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(ur3_driver_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/ur3_driver)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/ur3_driver
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(ur3_driver_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/ur3_driver)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/ur3_driver
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(ur3_driver_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ur3_driver)
  install(CODE "execute_process(COMMAND \"/usr/bin/python2\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ur3_driver\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ur3_driver
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(ur3_driver_generate_messages_py std_msgs_generate_messages_py)
endif()
