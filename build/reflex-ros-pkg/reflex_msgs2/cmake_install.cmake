# Install script for directory: /home/cc/ee106a/fa18/class/ee106a-aab/ros_workspaces/TheCreator/src/reflex-ros-pkg/reflex_msgs2

# Set the install prefix
IF(NOT DEFINED CMAKE_INSTALL_PREFIX)
  SET(CMAKE_INSTALL_PREFIX "/home/cc/ee106a/fa18/class/ee106a-aab/ros_workspaces/TheCreator/install")
ENDIF(NOT DEFINED CMAKE_INSTALL_PREFIX)
STRING(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
IF(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  IF(BUILD_TYPE)
    STRING(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  ELSE(BUILD_TYPE)
    SET(CMAKE_INSTALL_CONFIG_NAME "")
  ENDIF(BUILD_TYPE)
  MESSAGE(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
ENDIF(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)

# Set the component getting installed.
IF(NOT CMAKE_INSTALL_COMPONENT)
  IF(COMPONENT)
    MESSAGE(STATUS "Install component: \"${COMPONENT}\"")
    SET(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  ELSE(COMPONENT)
    SET(CMAKE_INSTALL_COMPONENT)
  ENDIF(COMPONENT)
ENDIF(NOT CMAKE_INSTALL_COMPONENT)

# Install shared libraries without execute permission?
IF(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  SET(CMAKE_INSTALL_SO_NO_EXE "1")
ENDIF(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/reflex_msgs2/msg" TYPE FILE FILES
    "/home/cc/ee106a/fa18/class/ee106a-aab/ros_workspaces/TheCreator/src/reflex-ros-pkg/reflex_msgs2/msg/Hand.msg"
    "/home/cc/ee106a/fa18/class/ee106a-aab/ros_workspaces/TheCreator/src/reflex-ros-pkg/reflex_msgs2/msg/Imu.msg"
    "/home/cc/ee106a/fa18/class/ee106a-aab/ros_workspaces/TheCreator/src/reflex-ros-pkg/reflex_msgs2/msg/Finger.msg"
    "/home/cc/ee106a/fa18/class/ee106a-aab/ros_workspaces/TheCreator/src/reflex-ros-pkg/reflex_msgs2/msg/FingerPressure.msg"
    "/home/cc/ee106a/fa18/class/ee106a-aab/ros_workspaces/TheCreator/src/reflex-ros-pkg/reflex_msgs2/msg/Motor.msg"
    "/home/cc/ee106a/fa18/class/ee106a-aab/ros_workspaces/TheCreator/src/reflex-ros-pkg/reflex_msgs2/msg/Command.msg"
    "/home/cc/ee106a/fa18/class/ee106a-aab/ros_workspaces/TheCreator/src/reflex-ros-pkg/reflex_msgs2/msg/PoseCommand.msg"
    "/home/cc/ee106a/fa18/class/ee106a-aab/ros_workspaces/TheCreator/src/reflex-ros-pkg/reflex_msgs2/msg/VelocityCommand.msg"
    "/home/cc/ee106a/fa18/class/ee106a-aab/ros_workspaces/TheCreator/src/reflex-ros-pkg/reflex_msgs2/msg/ForceCommand.msg"
    "/home/cc/ee106a/fa18/class/ee106a-aab/ros_workspaces/TheCreator/src/reflex-ros-pkg/reflex_msgs2/msg/RawServoCommands.msg"
    "/home/cc/ee106a/fa18/class/ee106a-aab/ros_workspaces/TheCreator/src/reflex-ros-pkg/reflex_msgs2/msg/RadianServoCommands.msg"
    "/home/cc/ee106a/fa18/class/ee106a-aab/ros_workspaces/TheCreator/src/reflex-ros-pkg/reflex_msgs2/msg/ImuCalibrationData.msg"
    )
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/reflex_msgs2/srv" TYPE FILE FILES
    "/home/cc/ee106a/fa18/class/ee106a-aab/ros_workspaces/TheCreator/src/reflex-ros-pkg/reflex_msgs2/srv/SetSpeed.srv"
    "/home/cc/ee106a/fa18/class/ee106a-aab/ros_workspaces/TheCreator/src/reflex-ros-pkg/reflex_msgs2/srv/SetTactileThreshold.srv"
    "/home/cc/ee106a/fa18/class/ee106a-aab/ros_workspaces/TheCreator/src/reflex-ros-pkg/reflex_msgs2/srv/DistalRotation.srv"
    )
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/reflex_msgs2/cmake" TYPE FILE FILES "/home/cc/ee106a/fa18/class/ee106a-aab/ros_workspaces/TheCreator/build/reflex-ros-pkg/reflex_msgs2/catkin_generated/installspace/reflex_msgs2-msg-paths.cmake")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/cc/ee106a/fa18/class/ee106a-aab/ros_workspaces/TheCreator/devel/include/reflex_msgs2")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES "/home/cc/ee106a/fa18/class/ee106a-aab/ros_workspaces/TheCreator/devel/share/common-lisp/ros/reflex_msgs2")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  execute_process(COMMAND "/usr/bin/python" -m compileall "/home/cc/ee106a/fa18/class/ee106a-aab/ros_workspaces/TheCreator/devel/lib/python2.7/dist-packages/reflex_msgs2")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python2.7/dist-packages" TYPE DIRECTORY FILES "/home/cc/ee106a/fa18/class/ee106a-aab/ros_workspaces/TheCreator/devel/lib/python2.7/dist-packages/reflex_msgs2")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/cc/ee106a/fa18/class/ee106a-aab/ros_workspaces/TheCreator/build/reflex-ros-pkg/reflex_msgs2/catkin_generated/installspace/reflex_msgs2.pc")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/reflex_msgs2/cmake" TYPE FILE FILES "/home/cc/ee106a/fa18/class/ee106a-aab/ros_workspaces/TheCreator/build/reflex-ros-pkg/reflex_msgs2/catkin_generated/installspace/reflex_msgs2-msg-extras.cmake")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/reflex_msgs2/cmake" TYPE FILE FILES
    "/home/cc/ee106a/fa18/class/ee106a-aab/ros_workspaces/TheCreator/build/reflex-ros-pkg/reflex_msgs2/catkin_generated/installspace/reflex_msgs2Config.cmake"
    "/home/cc/ee106a/fa18/class/ee106a-aab/ros_workspaces/TheCreator/build/reflex-ros-pkg/reflex_msgs2/catkin_generated/installspace/reflex_msgs2Config-version.cmake"
    )
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/reflex_msgs2" TYPE FILE FILES "/home/cc/ee106a/fa18/class/ee106a-aab/ros_workspaces/TheCreator/src/reflex-ros-pkg/reflex_msgs2/package.xml")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

