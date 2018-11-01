#!/usr/bin/python
################################################################################
#
# Node to wrap the Grasping.py class
#
################################################################################

from grasping import Grasping

import rospy
import sys

if __name__ == "__main__":
    rospy.init_node("grasping")

    g = Grasping()
    if not g.Initialize():
        rospy.logerr("Failed to initialize the grasping node.")
        sys.exit(1)

    rospy.spin()
