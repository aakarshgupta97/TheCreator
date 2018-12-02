#!/usr/bin/env python
"""
Path Planning Script for Creator
Authors: Aakarsh Gupta, Amay Saxena, Sairanjith Thalanki
"""

import sys
import rospy
import numpy as np
import controller

from moveit_msgs.msg import OrientationConstraint
from geometry_msgs.msg import PoseStamped

from path_planner import PathPlanner
import baxter_interface
from baxter_interface import gripper as robot_gripper

import threading


BLOCK_SIDE_LENGTH = 0.062
BLOCK_HEIGHT = 0.018
BLOCK_TOTAL_HEIGHT = 0.04
GRIPPER_LENGTH = 0.15


def add_block_obstacle(planner, position, name, 
    size=[BLOCK_SIDE_LENGTH, BLOCK_SIDE_LENGTH, BLOCK_TOTAL_HEIGHT], orient = [0.0, 0.0, 0.0, 1.0]):

    obstacle = PoseStamped()
    obstacle.pose.position.x = position[0]
    obstacle.pose.position.y = position[1]
    obstacle.pose.position.z = position[2]

    obstacle.pose.orientation.x = orient[0]
    obstacle.pose.orientation.y = orient[1]
    obstacle.pose.orientation.z = orient[2]
    obstacle.pose.orientation.w = orient[3]

    obstacle.header.frame_id = "base"
    planner.add_box_obstacle(size, name, obstacle)


def move_arm_to(x, y, z, ox, oy, oz, ow, orient_constraint=True):
    planner = PathPlanner("right_arm")
    
    while not rospy.is_shutdown():
        try:

            pick = PoseStamped()
            pick.header.frame_id = "base"

            #x, y, and z position
            pick.pose.position.x = x
            pick.pose.position.y = y
            pick.pose.position.z = z

            #Orientation as a quaternion
            pick.pose.orientation.x = ox
            pick.pose.orientation.y = oy
            pick.pose.orientation.z = oz
            pick.pose.orientation.w = ow

            # cont = controller.Controller(Kp, Kd, Ki, Kw, Limb('right_arm'))
            if orient_constraint:
                plan = planner.plan_to_pose(pick, [orient_constraint])
            else:
                plan = planner.plan_to_pose(pick, list())

            # if not cont.execute_path(plan);
            if not planner.execute_plan(plan):
                raise Exception()
        except Exception as e:
            print("Exception has occured: ", str(e))
            continue
        else:
            break

def inflate_gripper(gripper, stopLock):
    while stopLock.locked():
        gripper.inflate()
        rospy.sleep(0.5)

def main():

    # Set up the right gripper
    right_gripper = robot_gripper.Gripper('right')

    right_gripper.inflate = lambda: right_gripper.close()
    right_gripper.deflate = lambda: right_gripper.open()

    Kp = 0.1 * np.array([0.3, 2, 1, 1.5, 2, 2, 3]) # Stolen from 106B Students
    Kd = 0.01 * np.array([2, 1, 2, 0.5, 0.5, 0.5, 0.5]) # Stolen from 106B Students
    Ki = 0.01 * np.array([1, 1, 1, 1, 1, 1, 1]) # Untuned
    Kw = np.array([0.9, 0.9, 0.9, 0.9, 0.9, 0.9, 0.9]) # Untuned

    # Create a path constraint for the arm
    orient_const = OrientationConstraint()
    orient_const.link_name = "right_gripper"
    orient_const.header.frame_id = "base"
    orient_const.orientation.y = -1.0
    orient_const.absolute_x_axis_tolerance = 0.1
    orient_const.absolute_y_axis_tolerance = 0.1
    orient_const.absolute_z_axis_tolerance = 0.1
    orient_const.weight = 1.0

    top_left_x = 0.93
    top_left_y = -0.08

    start_x, start_y, start_z = 0.662, 0.039, 0.1


    raw_input("Press <Enter>")
    move_arm_to(0.583, -0.78, 0.08, 0, -1, 0, 0, False)

    right_gripper.inflate = lambda: right_gripper.close()
    right_gripper.deflate = lambda: right_gripper.open()
        
    stopLock = threading.Lock()
    stopLock.acquire()
    inflate_gripper_thread = threading.Thread(target=inflate_gripper, args=(right_gripper, stopLock))
    inflate_gripper_thread.start()

    raw_input("Press <Enter> to move the arm to pick up the block: ")
    move_arm_to(0.706, -0.575, 0.109, 0, -1, 0, 0)

    stopLock.release()
    inflate_gripper_thread.join()
   
    print("Done!")


if __name__ == '__main__':
    rospy.init_node('moveit_node')
    main()