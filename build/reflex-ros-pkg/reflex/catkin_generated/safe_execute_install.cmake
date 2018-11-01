execute_process(COMMAND "/home/cc/ee106a/fa18/class/ee106a-aab/ros_workspaces/TheCreator/build/reflex-ros-pkg/reflex/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/cc/ee106a/fa18/class/ee106a-aab/ros_workspaces/TheCreator/build/reflex-ros-pkg/reflex/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
