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
import intera_interface

BLOCK_SIDE_LENGTH = 0.062
BLOCK_HEIGHT = 0.018
BLOCK_TOTAL_HEIGHT = 0.04
GRIPPER_LENGTH = 

def inflate(gripper):
	print('Inflating...')
	gripper.close()
	rospy.sleep(2.0)
	print('Done!')

def deflate(gripper):
	print('Deflating...')
	gripper.open()
	rospy.sleep(2.0)
	print('Done!')

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

def block_coord_to_sawyer(block_coord_tl):

        ONE_STUD = BLOCK_SIDE_LENGTH / 2.0

        stud_x = block_coord_tl[0]
        stud_y = block_coord_tl[1]

        tl = (top_left_x - (ONE_STUD * stud_x), top_left_y - (ONE_STUD * stud_y))
        tr = (top_left_x - (ONE_STUD * (stud_x + 1)), top_left_y - (ONE_STUD * stud_y))

        bl = (top_left_x - (ONE_STUD * stud_x), top_left_y - (ONE_STUD * (stud_y+1)))
        br = (top_left_x - (ONE_STUD * (stud_x + 1)), top_left_y - (ONE_STUD * (stud_y+1)))

        return [0.5 * (tl[0] + tr[0]), 0.5 * (tl[1] + bl[1])]


def move_arm_to(x,y,z, ox, oy, oz, ow, orient_constraint=True):
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
                plan = planner.plan_to_pose(pick, [orien_const])
            else:
                plan = planner.plan_to_pose(pick, list())

            # if not cont.execute_path(plan);
            if not planner.execute_plan(plan):
                raise Exception()
        except Exception as e:
            print("Exception has occured")
            continue
        else:
            break

def main():

    planner = PathPlanner("right_arm")

    # Set up the right gripper
    gripper = intera_interface.Gripper('right_gripper')

    # Calibrate the gripper
    # print('Calibrating...')
    # gripper.calibrate()
    # rospy.sleep(2.0)

    Kp = 0.1 * np.array([0.3, 2, 1, 1.5, 2, 2, 3]) # Stolen from 106B Students
    Kd = 0.01 * np.array([2, 1, 2, 0.5, 0.5, 0.5, 0.5]) # Stolen from 106B Students
    Ki = 0.01 * np.array([1, 1, 1, 1, 1, 1, 1]) # Untuned
    Kw = np.array([0.9, 0.9, 0.9, 0.9, 0.9, 0.9, 0.9]) # Untuned

    # Create a path constraint for the arm
    orien_const = OrientationConstraint()
    orien_const.link_name = "right_gripper"
    orien_const.header.frame_id = "base"
    orien_const.orientation.y = -1.0
    orien_const.absolute_x_axis_tolerance = 0.1
    orien_const.absolute_y_axis_tolerance = 0.1
    orien_const.absolute_z_axis_tolerance = 0.1
    orien_const.weight = 1.0

    # Surrounding objects for Sawyer to plan around
    add_block_obstacle(planner, [0.303, 0.79, 0.67], 'left_wall', [10, 10, 0], [0.56, 0.0, 0.0, 1.0])
    add_block_obstacle(planner, [0,0,0], 'ceiling', [10, 5, 1])
    add_block_obstacle(planner, [0.963, -0.15, -0.1], 'table', [10, 10, 0])
    add_block_obstacle(planner, [0.81, -0.26, -0.1 + (0.5 * BLOCK_TOTAL_HEIGHT)], 'baseplate', [5 * BLOCK_SIDE_LENGTH, 6 * BLOCK_SIDE_LENGTH , 0.5 * BLOCK_TOTAL_HEIGHT])
    
    top_left_x = 0.93
    top_left_y = -0.08

    start_x, start_y, start_z = 0.662, 0.039, 0.1

    def pick_and_place_block(dest_x, dest_y, dest_z, block_name):

    	raw_input("Press <Enter> to move the arm to x and y of pick up block: ")
    	move_arm_to(start_x, start_y, start_z, 0, -1, 0, 0, False)

    	raw_input("Press <Enter> to move the arm to pick up the block: ")
    	move_arm_to(start_x, start_y, -0.07, 0, -1, 0, 0)
        
        raw_input("Press <Enter> to move the arm to move up with the block: ")
        move_arm_to(start_x, start_y, start_z, 0, -1, 0, 0, False)

        raw_input("Press <Enter> to move the arm to x and y destinations: ")
        move_arm_to(dest_x, dest_y, 0.1, 0, -1, 0, 0, False)

        raw_input("Press <Enter> to drop the block: ")
        move_arm_to(dest_x, dest_y, 0.02 + dest_z, 0, -1, 0, 0) # Add 0.02 to avoid collision. Let gravity do the rest.

        raw_input("Press <Enter> to come up: ")
        move_arm_to(dest_x, dest_y, 0.1, 0, -1, 0, 0, False)

        raw_input("Press <Enter> to move to starting position: ")
        move_arm_to(start_x, start_y, start_z, 0, -1, 0, 0, False)

        add_block_obstacle(planner, [dest_x, dest_y, dest_z], block_name, [0.5 * BLOCK_SIDE_LENGTH, 0.5 * BLOCK_SIDE_LENGTH , 0.5 * BLOCK_TOTAL_HEIGHT])
   

    # print(block_coord_to_sawyer((6, 5)))
    pick_and_place_block(0.81, -0.27, -0.07, 'red_block')

    raw_input("Press <Enter> to place next block: ")

    pick_and_place_block(0.86, -0.27, -0.07, 'yellow_block')

    raw_input("Press <Enter> to place next block: ")

    pick_and_place_block(0.83, -0.2, -0.07, 'next_block')

    raw_input("Press <Enter> to terminate: ")
    print("Done!")


if __name__ == '__main__':
    rospy.init_node('moveit_node')
    main()
