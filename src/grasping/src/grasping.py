################################################################################
#
# Grasping class listens to interface with the RightHand Reflex SF grasping hand
#
################################################################################

import rospy
import tf2_ros
import tf

from geometry_msgs.msg import Point

import numpy as np

class Grasping:
	def __init__(self):
        self._intialized = False

        # Set up tf buffer and listener.
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer)

    # Initialization and loading parameters.
    def Initialize(self):
        self._name = rospy.get_name() + "/grid_map_2d"

        # Load parameters.
        if not self.LoadParameters():
            rospy.logerr("%s: Error loading parameters.", self._name)
            return False

        # Register callbacks.
        if not self.RegisterCallbacks():
            rospy.logerr("%s: Error registering callbacks.", self._name)
            return False

        # Set up data structure
        """TODO"""

        self._initialized = True
        return True

    def LoadParameters(self):
    	return True

    def RegisterCallbacks(self):
    	return True
