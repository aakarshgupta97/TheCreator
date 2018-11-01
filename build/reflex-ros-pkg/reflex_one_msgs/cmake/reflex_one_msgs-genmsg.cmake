# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "reflex_one_msgs: 10 messages, 2 services")

set(MSG_I_FLAGS "-Ireflex_one_msgs:/home/cc/ee106a/fa18/class/ee106a-aab/ros_workspaces/TheCreator/src/reflex-ros-pkg/reflex_one_msgs/msg;-Istd_msgs:/opt/ros/indigo/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(genlisp REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(reflex_one_msgs_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/cc/ee106a/fa18/class/ee106a-aab/ros_workspaces/TheCreator/src/reflex-ros-pkg/reflex_one_msgs/msg/FingerPressure.msg" NAME_WE)
add_custom_target(_reflex_one_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "reflex_one_msgs" "/home/cc/ee106a/fa18/class/ee106a-aab/ros_workspaces/TheCreator/src/reflex-ros-pkg/reflex_one_msgs/msg/FingerPressure.msg" ""
)

get_filename_component(_filename "/home/cc/ee106a/fa18/class/ee106a-aab/ros_workspaces/TheCreator/src/reflex-ros-pkg/reflex_one_msgs/msg/PoseCommand.msg" NAME_WE)
add_custom_target(_reflex_one_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "reflex_one_msgs" "/home/cc/ee106a/fa18/class/ee106a-aab/ros_workspaces/TheCreator/src/reflex-ros-pkg/reflex_one_msgs/msg/PoseCommand.msg" ""
)

get_filename_component(_filename "/home/cc/ee106a/fa18/class/ee106a-aab/ros_workspaces/TheCreator/src/reflex-ros-pkg/reflex_one_msgs/msg/RadianServoCommands.msg" NAME_WE)
add_custom_target(_reflex_one_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "reflex_one_msgs" "/home/cc/ee106a/fa18/class/ee106a-aab/ros_workspaces/TheCreator/src/reflex-ros-pkg/reflex_one_msgs/msg/RadianServoCommands.msg" ""
)

get_filename_component(_filename "/home/cc/ee106a/fa18/class/ee106a-aab/ros_workspaces/TheCreator/src/reflex-ros-pkg/reflex_one_msgs/msg/ForceCommand.msg" NAME_WE)
add_custom_target(_reflex_one_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "reflex_one_msgs" "/home/cc/ee106a/fa18/class/ee106a-aab/ros_workspaces/TheCreator/src/reflex-ros-pkg/reflex_one_msgs/msg/ForceCommand.msg" ""
)

get_filename_component(_filename "/home/cc/ee106a/fa18/class/ee106a-aab/ros_workspaces/TheCreator/src/reflex-ros-pkg/reflex_one_msgs/srv/SetTactileThreshold.srv" NAME_WE)
add_custom_target(_reflex_one_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "reflex_one_msgs" "/home/cc/ee106a/fa18/class/ee106a-aab/ros_workspaces/TheCreator/src/reflex-ros-pkg/reflex_one_msgs/srv/SetTactileThreshold.srv" "reflex_one_msgs/FingerPressure"
)

get_filename_component(_filename "/home/cc/ee106a/fa18/class/ee106a-aab/ros_workspaces/TheCreator/src/reflex-ros-pkg/reflex_one_msgs/msg/Motor.msg" NAME_WE)
add_custom_target(_reflex_one_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "reflex_one_msgs" "/home/cc/ee106a/fa18/class/ee106a-aab/ros_workspaces/TheCreator/src/reflex-ros-pkg/reflex_one_msgs/msg/Motor.msg" ""
)

get_filename_component(_filename "/home/cc/ee106a/fa18/class/ee106a-aab/ros_workspaces/TheCreator/src/reflex-ros-pkg/reflex_one_msgs/msg/VelocityCommand.msg" NAME_WE)
add_custom_target(_reflex_one_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "reflex_one_msgs" "/home/cc/ee106a/fa18/class/ee106a-aab/ros_workspaces/TheCreator/src/reflex-ros-pkg/reflex_one_msgs/msg/VelocityCommand.msg" ""
)

get_filename_component(_filename "/home/cc/ee106a/fa18/class/ee106a-aab/ros_workspaces/TheCreator/src/reflex-ros-pkg/reflex_one_msgs/msg/Command.msg" NAME_WE)
add_custom_target(_reflex_one_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "reflex_one_msgs" "/home/cc/ee106a/fa18/class/ee106a-aab/ros_workspaces/TheCreator/src/reflex-ros-pkg/reflex_one_msgs/msg/Command.msg" "reflex_one_msgs/PoseCommand:reflex_one_msgs/VelocityCommand"
)

get_filename_component(_filename "/home/cc/ee106a/fa18/class/ee106a-aab/ros_workspaces/TheCreator/src/reflex-ros-pkg/reflex_one_msgs/msg/RawServoCommands.msg" NAME_WE)
add_custom_target(_reflex_one_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "reflex_one_msgs" "/home/cc/ee106a/fa18/class/ee106a-aab/ros_workspaces/TheCreator/src/reflex-ros-pkg/reflex_one_msgs/msg/RawServoCommands.msg" ""
)

get_filename_component(_filename "/home/cc/ee106a/fa18/class/ee106a-aab/ros_workspaces/TheCreator/src/reflex-ros-pkg/reflex_one_msgs/srv/SetSpeed.srv" NAME_WE)
add_custom_target(_reflex_one_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "reflex_one_msgs" "/home/cc/ee106a/fa18/class/ee106a-aab/ros_workspaces/TheCreator/src/reflex-ros-pkg/reflex_one_msgs/srv/SetSpeed.srv" ""
)

get_filename_component(_filename "/home/cc/ee106a/fa18/class/ee106a-aab/ros_workspaces/TheCreator/src/reflex-ros-pkg/reflex_one_msgs/msg/Finger.msg" NAME_WE)
add_custom_target(_reflex_one_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "reflex_one_msgs" "/home/cc/ee106a/fa18/class/ee106a-aab/ros_workspaces/TheCreator/src/reflex-ros-pkg/reflex_one_msgs/msg/Finger.msg" ""
)

get_filename_component(_filename "/home/cc/ee106a/fa18/class/ee106a-aab/ros_workspaces/TheCreator/src/reflex-ros-pkg/reflex_one_msgs/msg/Hand.msg" NAME_WE)
add_custom_target(_reflex_one_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "reflex_one_msgs" "/home/cc/ee106a/fa18/class/ee106a-aab/ros_workspaces/TheCreator/src/reflex-ros-pkg/reflex_one_msgs/msg/Hand.msg" "reflex_one_msgs/Motor:reflex_one_msgs/Finger"
)

#
#  langs = gencpp;genlisp;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(reflex_one_msgs
  "/home/cc/ee106a/fa18/class/ee106a-aab/ros_workspaces/TheCreator/src/reflex-ros-pkg/reflex_one_msgs/msg/FingerPressure.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/reflex_one_msgs
)
_generate_msg_cpp(reflex_one_msgs
  "/home/cc/ee106a/fa18/class/ee106a-aab/ros_workspaces/TheCreator/src/reflex-ros-pkg/reflex_one_msgs/msg/PoseCommand.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/reflex_one_msgs
)
_generate_msg_cpp(reflex_one_msgs
  "/home/cc/ee106a/fa18/class/ee106a-aab/ros_workspaces/TheCreator/src/reflex-ros-pkg/reflex_one_msgs/msg/RadianServoCommands.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/reflex_one_msgs
)
_generate_msg_cpp(reflex_one_msgs
  "/home/cc/ee106a/fa18/class/ee106a-aab/ros_workspaces/TheCreator/src/reflex-ros-pkg/reflex_one_msgs/msg/ForceCommand.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/reflex_one_msgs
)
_generate_msg_cpp(reflex_one_msgs
  "/home/cc/ee106a/fa18/class/ee106a-aab/ros_workspaces/TheCreator/src/reflex-ros-pkg/reflex_one_msgs/msg/Motor.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/reflex_one_msgs
)
_generate_msg_cpp(reflex_one_msgs
  "/home/cc/ee106a/fa18/class/ee106a-aab/ros_workspaces/TheCreator/src/reflex-ros-pkg/reflex_one_msgs/msg/VelocityCommand.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/reflex_one_msgs
)
_generate_msg_cpp(reflex_one_msgs
  "/home/cc/ee106a/fa18/class/ee106a-aab/ros_workspaces/TheCreator/src/reflex-ros-pkg/reflex_one_msgs/msg/Command.msg"
  "${MSG_I_FLAGS}"
  "/home/cc/ee106a/fa18/class/ee106a-aab/ros_workspaces/TheCreator/src/reflex-ros-pkg/reflex_one_msgs/msg/PoseCommand.msg;/home/cc/ee106a/fa18/class/ee106a-aab/ros_workspaces/TheCreator/src/reflex-ros-pkg/reflex_one_msgs/msg/VelocityCommand.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/reflex_one_msgs
)
_generate_msg_cpp(reflex_one_msgs
  "/home/cc/ee106a/fa18/class/ee106a-aab/ros_workspaces/TheCreator/src/reflex-ros-pkg/reflex_one_msgs/msg/RawServoCommands.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/reflex_one_msgs
)
_generate_msg_cpp(reflex_one_msgs
  "/home/cc/ee106a/fa18/class/ee106a-aab/ros_workspaces/TheCreator/src/reflex-ros-pkg/reflex_one_msgs/msg/Finger.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/reflex_one_msgs
)
_generate_msg_cpp(reflex_one_msgs
  "/home/cc/ee106a/fa18/class/ee106a-aab/ros_workspaces/TheCreator/src/reflex-ros-pkg/reflex_one_msgs/msg/Hand.msg"
  "${MSG_I_FLAGS}"
  "/home/cc/ee106a/fa18/class/ee106a-aab/ros_workspaces/TheCreator/src/reflex-ros-pkg/reflex_one_msgs/msg/Motor.msg;/home/cc/ee106a/fa18/class/ee106a-aab/ros_workspaces/TheCreator/src/reflex-ros-pkg/reflex_one_msgs/msg/Finger.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/reflex_one_msgs
)

### Generating Services
_generate_srv_cpp(reflex_one_msgs
  "/home/cc/ee106a/fa18/class/ee106a-aab/ros_workspaces/TheCreator/src/reflex-ros-pkg/reflex_one_msgs/srv/SetSpeed.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/reflex_one_msgs
)
_generate_srv_cpp(reflex_one_msgs
  "/home/cc/ee106a/fa18/class/ee106a-aab/ros_workspaces/TheCreator/src/reflex-ros-pkg/reflex_one_msgs/srv/SetTactileThreshold.srv"
  "${MSG_I_FLAGS}"
  "/home/cc/ee106a/fa18/class/ee106a-aab/ros_workspaces/TheCreator/src/reflex-ros-pkg/reflex_one_msgs/msg/FingerPressure.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/reflex_one_msgs
)

### Generating Module File
_generate_module_cpp(reflex_one_msgs
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/reflex_one_msgs
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(reflex_one_msgs_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(reflex_one_msgs_generate_messages reflex_one_msgs_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/cc/ee106a/fa18/class/ee106a-aab/ros_workspaces/TheCreator/src/reflex-ros-pkg/reflex_one_msgs/msg/FingerPressure.msg" NAME_WE)
add_dependencies(reflex_one_msgs_generate_messages_cpp _reflex_one_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cc/ee106a/fa18/class/ee106a-aab/ros_workspaces/TheCreator/src/reflex-ros-pkg/reflex_one_msgs/msg/PoseCommand.msg" NAME_WE)
add_dependencies(reflex_one_msgs_generate_messages_cpp _reflex_one_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cc/ee106a/fa18/class/ee106a-aab/ros_workspaces/TheCreator/src/reflex-ros-pkg/reflex_one_msgs/msg/RadianServoCommands.msg" NAME_WE)
add_dependencies(reflex_one_msgs_generate_messages_cpp _reflex_one_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cc/ee106a/fa18/class/ee106a-aab/ros_workspaces/TheCreator/src/reflex-ros-pkg/reflex_one_msgs/msg/ForceCommand.msg" NAME_WE)
add_dependencies(reflex_one_msgs_generate_messages_cpp _reflex_one_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cc/ee106a/fa18/class/ee106a-aab/ros_workspaces/TheCreator/src/reflex-ros-pkg/reflex_one_msgs/srv/SetTactileThreshold.srv" NAME_WE)
add_dependencies(reflex_one_msgs_generate_messages_cpp _reflex_one_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cc/ee106a/fa18/class/ee106a-aab/ros_workspaces/TheCreator/src/reflex-ros-pkg/reflex_one_msgs/msg/Motor.msg" NAME_WE)
add_dependencies(reflex_one_msgs_generate_messages_cpp _reflex_one_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cc/ee106a/fa18/class/ee106a-aab/ros_workspaces/TheCreator/src/reflex-ros-pkg/reflex_one_msgs/msg/VelocityCommand.msg" NAME_WE)
add_dependencies(reflex_one_msgs_generate_messages_cpp _reflex_one_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cc/ee106a/fa18/class/ee106a-aab/ros_workspaces/TheCreator/src/reflex-ros-pkg/reflex_one_msgs/msg/Command.msg" NAME_WE)
add_dependencies(reflex_one_msgs_generate_messages_cpp _reflex_one_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cc/ee106a/fa18/class/ee106a-aab/ros_workspaces/TheCreator/src/reflex-ros-pkg/reflex_one_msgs/msg/RawServoCommands.msg" NAME_WE)
add_dependencies(reflex_one_msgs_generate_messages_cpp _reflex_one_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cc/ee106a/fa18/class/ee106a-aab/ros_workspaces/TheCreator/src/reflex-ros-pkg/reflex_one_msgs/srv/SetSpeed.srv" NAME_WE)
add_dependencies(reflex_one_msgs_generate_messages_cpp _reflex_one_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cc/ee106a/fa18/class/ee106a-aab/ros_workspaces/TheCreator/src/reflex-ros-pkg/reflex_one_msgs/msg/Finger.msg" NAME_WE)
add_dependencies(reflex_one_msgs_generate_messages_cpp _reflex_one_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cc/ee106a/fa18/class/ee106a-aab/ros_workspaces/TheCreator/src/reflex-ros-pkg/reflex_one_msgs/msg/Hand.msg" NAME_WE)
add_dependencies(reflex_one_msgs_generate_messages_cpp _reflex_one_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(reflex_one_msgs_gencpp)
add_dependencies(reflex_one_msgs_gencpp reflex_one_msgs_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS reflex_one_msgs_generate_messages_cpp)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(reflex_one_msgs
  "/home/cc/ee106a/fa18/class/ee106a-aab/ros_workspaces/TheCreator/src/reflex-ros-pkg/reflex_one_msgs/msg/FingerPressure.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/reflex_one_msgs
)
_generate_msg_lisp(reflex_one_msgs
  "/home/cc/ee106a/fa18/class/ee106a-aab/ros_workspaces/TheCreator/src/reflex-ros-pkg/reflex_one_msgs/msg/PoseCommand.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/reflex_one_msgs
)
_generate_msg_lisp(reflex_one_msgs
  "/home/cc/ee106a/fa18/class/ee106a-aab/ros_workspaces/TheCreator/src/reflex-ros-pkg/reflex_one_msgs/msg/RadianServoCommands.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/reflex_one_msgs
)
_generate_msg_lisp(reflex_one_msgs
  "/home/cc/ee106a/fa18/class/ee106a-aab/ros_workspaces/TheCreator/src/reflex-ros-pkg/reflex_one_msgs/msg/ForceCommand.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/reflex_one_msgs
)
_generate_msg_lisp(reflex_one_msgs
  "/home/cc/ee106a/fa18/class/ee106a-aab/ros_workspaces/TheCreator/src/reflex-ros-pkg/reflex_one_msgs/msg/Motor.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/reflex_one_msgs
)
_generate_msg_lisp(reflex_one_msgs
  "/home/cc/ee106a/fa18/class/ee106a-aab/ros_workspaces/TheCreator/src/reflex-ros-pkg/reflex_one_msgs/msg/VelocityCommand.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/reflex_one_msgs
)
_generate_msg_lisp(reflex_one_msgs
  "/home/cc/ee106a/fa18/class/ee106a-aab/ros_workspaces/TheCreator/src/reflex-ros-pkg/reflex_one_msgs/msg/Command.msg"
  "${MSG_I_FLAGS}"
  "/home/cc/ee106a/fa18/class/ee106a-aab/ros_workspaces/TheCreator/src/reflex-ros-pkg/reflex_one_msgs/msg/PoseCommand.msg;/home/cc/ee106a/fa18/class/ee106a-aab/ros_workspaces/TheCreator/src/reflex-ros-pkg/reflex_one_msgs/msg/VelocityCommand.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/reflex_one_msgs
)
_generate_msg_lisp(reflex_one_msgs
  "/home/cc/ee106a/fa18/class/ee106a-aab/ros_workspaces/TheCreator/src/reflex-ros-pkg/reflex_one_msgs/msg/RawServoCommands.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/reflex_one_msgs
)
_generate_msg_lisp(reflex_one_msgs
  "/home/cc/ee106a/fa18/class/ee106a-aab/ros_workspaces/TheCreator/src/reflex-ros-pkg/reflex_one_msgs/msg/Finger.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/reflex_one_msgs
)
_generate_msg_lisp(reflex_one_msgs
  "/home/cc/ee106a/fa18/class/ee106a-aab/ros_workspaces/TheCreator/src/reflex-ros-pkg/reflex_one_msgs/msg/Hand.msg"
  "${MSG_I_FLAGS}"
  "/home/cc/ee106a/fa18/class/ee106a-aab/ros_workspaces/TheCreator/src/reflex-ros-pkg/reflex_one_msgs/msg/Motor.msg;/home/cc/ee106a/fa18/class/ee106a-aab/ros_workspaces/TheCreator/src/reflex-ros-pkg/reflex_one_msgs/msg/Finger.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/reflex_one_msgs
)

### Generating Services
_generate_srv_lisp(reflex_one_msgs
  "/home/cc/ee106a/fa18/class/ee106a-aab/ros_workspaces/TheCreator/src/reflex-ros-pkg/reflex_one_msgs/srv/SetSpeed.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/reflex_one_msgs
)
_generate_srv_lisp(reflex_one_msgs
  "/home/cc/ee106a/fa18/class/ee106a-aab/ros_workspaces/TheCreator/src/reflex-ros-pkg/reflex_one_msgs/srv/SetTactileThreshold.srv"
  "${MSG_I_FLAGS}"
  "/home/cc/ee106a/fa18/class/ee106a-aab/ros_workspaces/TheCreator/src/reflex-ros-pkg/reflex_one_msgs/msg/FingerPressure.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/reflex_one_msgs
)

### Generating Module File
_generate_module_lisp(reflex_one_msgs
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/reflex_one_msgs
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(reflex_one_msgs_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(reflex_one_msgs_generate_messages reflex_one_msgs_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/cc/ee106a/fa18/class/ee106a-aab/ros_workspaces/TheCreator/src/reflex-ros-pkg/reflex_one_msgs/msg/FingerPressure.msg" NAME_WE)
add_dependencies(reflex_one_msgs_generate_messages_lisp _reflex_one_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cc/ee106a/fa18/class/ee106a-aab/ros_workspaces/TheCreator/src/reflex-ros-pkg/reflex_one_msgs/msg/PoseCommand.msg" NAME_WE)
add_dependencies(reflex_one_msgs_generate_messages_lisp _reflex_one_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cc/ee106a/fa18/class/ee106a-aab/ros_workspaces/TheCreator/src/reflex-ros-pkg/reflex_one_msgs/msg/RadianServoCommands.msg" NAME_WE)
add_dependencies(reflex_one_msgs_generate_messages_lisp _reflex_one_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cc/ee106a/fa18/class/ee106a-aab/ros_workspaces/TheCreator/src/reflex-ros-pkg/reflex_one_msgs/msg/ForceCommand.msg" NAME_WE)
add_dependencies(reflex_one_msgs_generate_messages_lisp _reflex_one_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cc/ee106a/fa18/class/ee106a-aab/ros_workspaces/TheCreator/src/reflex-ros-pkg/reflex_one_msgs/srv/SetTactileThreshold.srv" NAME_WE)
add_dependencies(reflex_one_msgs_generate_messages_lisp _reflex_one_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cc/ee106a/fa18/class/ee106a-aab/ros_workspaces/TheCreator/src/reflex-ros-pkg/reflex_one_msgs/msg/Motor.msg" NAME_WE)
add_dependencies(reflex_one_msgs_generate_messages_lisp _reflex_one_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cc/ee106a/fa18/class/ee106a-aab/ros_workspaces/TheCreator/src/reflex-ros-pkg/reflex_one_msgs/msg/VelocityCommand.msg" NAME_WE)
add_dependencies(reflex_one_msgs_generate_messages_lisp _reflex_one_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cc/ee106a/fa18/class/ee106a-aab/ros_workspaces/TheCreator/src/reflex-ros-pkg/reflex_one_msgs/msg/Command.msg" NAME_WE)
add_dependencies(reflex_one_msgs_generate_messages_lisp _reflex_one_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cc/ee106a/fa18/class/ee106a-aab/ros_workspaces/TheCreator/src/reflex-ros-pkg/reflex_one_msgs/msg/RawServoCommands.msg" NAME_WE)
add_dependencies(reflex_one_msgs_generate_messages_lisp _reflex_one_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cc/ee106a/fa18/class/ee106a-aab/ros_workspaces/TheCreator/src/reflex-ros-pkg/reflex_one_msgs/srv/SetSpeed.srv" NAME_WE)
add_dependencies(reflex_one_msgs_generate_messages_lisp _reflex_one_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cc/ee106a/fa18/class/ee106a-aab/ros_workspaces/TheCreator/src/reflex-ros-pkg/reflex_one_msgs/msg/Finger.msg" NAME_WE)
add_dependencies(reflex_one_msgs_generate_messages_lisp _reflex_one_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cc/ee106a/fa18/class/ee106a-aab/ros_workspaces/TheCreator/src/reflex-ros-pkg/reflex_one_msgs/msg/Hand.msg" NAME_WE)
add_dependencies(reflex_one_msgs_generate_messages_lisp _reflex_one_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(reflex_one_msgs_genlisp)
add_dependencies(reflex_one_msgs_genlisp reflex_one_msgs_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS reflex_one_msgs_generate_messages_lisp)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(reflex_one_msgs
  "/home/cc/ee106a/fa18/class/ee106a-aab/ros_workspaces/TheCreator/src/reflex-ros-pkg/reflex_one_msgs/msg/FingerPressure.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/reflex_one_msgs
)
_generate_msg_py(reflex_one_msgs
  "/home/cc/ee106a/fa18/class/ee106a-aab/ros_workspaces/TheCreator/src/reflex-ros-pkg/reflex_one_msgs/msg/PoseCommand.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/reflex_one_msgs
)
_generate_msg_py(reflex_one_msgs
  "/home/cc/ee106a/fa18/class/ee106a-aab/ros_workspaces/TheCreator/src/reflex-ros-pkg/reflex_one_msgs/msg/RadianServoCommands.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/reflex_one_msgs
)
_generate_msg_py(reflex_one_msgs
  "/home/cc/ee106a/fa18/class/ee106a-aab/ros_workspaces/TheCreator/src/reflex-ros-pkg/reflex_one_msgs/msg/ForceCommand.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/reflex_one_msgs
)
_generate_msg_py(reflex_one_msgs
  "/home/cc/ee106a/fa18/class/ee106a-aab/ros_workspaces/TheCreator/src/reflex-ros-pkg/reflex_one_msgs/msg/Motor.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/reflex_one_msgs
)
_generate_msg_py(reflex_one_msgs
  "/home/cc/ee106a/fa18/class/ee106a-aab/ros_workspaces/TheCreator/src/reflex-ros-pkg/reflex_one_msgs/msg/VelocityCommand.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/reflex_one_msgs
)
_generate_msg_py(reflex_one_msgs
  "/home/cc/ee106a/fa18/class/ee106a-aab/ros_workspaces/TheCreator/src/reflex-ros-pkg/reflex_one_msgs/msg/Command.msg"
  "${MSG_I_FLAGS}"
  "/home/cc/ee106a/fa18/class/ee106a-aab/ros_workspaces/TheCreator/src/reflex-ros-pkg/reflex_one_msgs/msg/PoseCommand.msg;/home/cc/ee106a/fa18/class/ee106a-aab/ros_workspaces/TheCreator/src/reflex-ros-pkg/reflex_one_msgs/msg/VelocityCommand.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/reflex_one_msgs
)
_generate_msg_py(reflex_one_msgs
  "/home/cc/ee106a/fa18/class/ee106a-aab/ros_workspaces/TheCreator/src/reflex-ros-pkg/reflex_one_msgs/msg/RawServoCommands.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/reflex_one_msgs
)
_generate_msg_py(reflex_one_msgs
  "/home/cc/ee106a/fa18/class/ee106a-aab/ros_workspaces/TheCreator/src/reflex-ros-pkg/reflex_one_msgs/msg/Finger.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/reflex_one_msgs
)
_generate_msg_py(reflex_one_msgs
  "/home/cc/ee106a/fa18/class/ee106a-aab/ros_workspaces/TheCreator/src/reflex-ros-pkg/reflex_one_msgs/msg/Hand.msg"
  "${MSG_I_FLAGS}"
  "/home/cc/ee106a/fa18/class/ee106a-aab/ros_workspaces/TheCreator/src/reflex-ros-pkg/reflex_one_msgs/msg/Motor.msg;/home/cc/ee106a/fa18/class/ee106a-aab/ros_workspaces/TheCreator/src/reflex-ros-pkg/reflex_one_msgs/msg/Finger.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/reflex_one_msgs
)

### Generating Services
_generate_srv_py(reflex_one_msgs
  "/home/cc/ee106a/fa18/class/ee106a-aab/ros_workspaces/TheCreator/src/reflex-ros-pkg/reflex_one_msgs/srv/SetSpeed.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/reflex_one_msgs
)
_generate_srv_py(reflex_one_msgs
  "/home/cc/ee106a/fa18/class/ee106a-aab/ros_workspaces/TheCreator/src/reflex-ros-pkg/reflex_one_msgs/srv/SetTactileThreshold.srv"
  "${MSG_I_FLAGS}"
  "/home/cc/ee106a/fa18/class/ee106a-aab/ros_workspaces/TheCreator/src/reflex-ros-pkg/reflex_one_msgs/msg/FingerPressure.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/reflex_one_msgs
)

### Generating Module File
_generate_module_py(reflex_one_msgs
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/reflex_one_msgs
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(reflex_one_msgs_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(reflex_one_msgs_generate_messages reflex_one_msgs_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/cc/ee106a/fa18/class/ee106a-aab/ros_workspaces/TheCreator/src/reflex-ros-pkg/reflex_one_msgs/msg/FingerPressure.msg" NAME_WE)
add_dependencies(reflex_one_msgs_generate_messages_py _reflex_one_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cc/ee106a/fa18/class/ee106a-aab/ros_workspaces/TheCreator/src/reflex-ros-pkg/reflex_one_msgs/msg/PoseCommand.msg" NAME_WE)
add_dependencies(reflex_one_msgs_generate_messages_py _reflex_one_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cc/ee106a/fa18/class/ee106a-aab/ros_workspaces/TheCreator/src/reflex-ros-pkg/reflex_one_msgs/msg/RadianServoCommands.msg" NAME_WE)
add_dependencies(reflex_one_msgs_generate_messages_py _reflex_one_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cc/ee106a/fa18/class/ee106a-aab/ros_workspaces/TheCreator/src/reflex-ros-pkg/reflex_one_msgs/msg/ForceCommand.msg" NAME_WE)
add_dependencies(reflex_one_msgs_generate_messages_py _reflex_one_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cc/ee106a/fa18/class/ee106a-aab/ros_workspaces/TheCreator/src/reflex-ros-pkg/reflex_one_msgs/srv/SetTactileThreshold.srv" NAME_WE)
add_dependencies(reflex_one_msgs_generate_messages_py _reflex_one_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cc/ee106a/fa18/class/ee106a-aab/ros_workspaces/TheCreator/src/reflex-ros-pkg/reflex_one_msgs/msg/Motor.msg" NAME_WE)
add_dependencies(reflex_one_msgs_generate_messages_py _reflex_one_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cc/ee106a/fa18/class/ee106a-aab/ros_workspaces/TheCreator/src/reflex-ros-pkg/reflex_one_msgs/msg/VelocityCommand.msg" NAME_WE)
add_dependencies(reflex_one_msgs_generate_messages_py _reflex_one_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cc/ee106a/fa18/class/ee106a-aab/ros_workspaces/TheCreator/src/reflex-ros-pkg/reflex_one_msgs/msg/Command.msg" NAME_WE)
add_dependencies(reflex_one_msgs_generate_messages_py _reflex_one_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cc/ee106a/fa18/class/ee106a-aab/ros_workspaces/TheCreator/src/reflex-ros-pkg/reflex_one_msgs/msg/RawServoCommands.msg" NAME_WE)
add_dependencies(reflex_one_msgs_generate_messages_py _reflex_one_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cc/ee106a/fa18/class/ee106a-aab/ros_workspaces/TheCreator/src/reflex-ros-pkg/reflex_one_msgs/srv/SetSpeed.srv" NAME_WE)
add_dependencies(reflex_one_msgs_generate_messages_py _reflex_one_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cc/ee106a/fa18/class/ee106a-aab/ros_workspaces/TheCreator/src/reflex-ros-pkg/reflex_one_msgs/msg/Finger.msg" NAME_WE)
add_dependencies(reflex_one_msgs_generate_messages_py _reflex_one_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cc/ee106a/fa18/class/ee106a-aab/ros_workspaces/TheCreator/src/reflex-ros-pkg/reflex_one_msgs/msg/Hand.msg" NAME_WE)
add_dependencies(reflex_one_msgs_generate_messages_py _reflex_one_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(reflex_one_msgs_genpy)
add_dependencies(reflex_one_msgs_genpy reflex_one_msgs_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS reflex_one_msgs_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/reflex_one_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/reflex_one_msgs
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(reflex_one_msgs_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/reflex_one_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/reflex_one_msgs
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(reflex_one_msgs_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/reflex_one_msgs)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/reflex_one_msgs\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/reflex_one_msgs
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(reflex_one_msgs_generate_messages_py std_msgs_generate_messages_py)
endif()
