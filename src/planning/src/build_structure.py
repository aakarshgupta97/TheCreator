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

import os
SAWYER_ROBOTS = ['ada', 'alan']
BAXTER_ROBOTS = ['asimov', 'aryton', 'archytas']
if any([robot_name in os.environ['ROS_MASTER_URI'] for robot_name in SAWYER_ROBOTS]):
	import intera_interface
	from intera_interface import Gripper
elif any([robot_name in os.environ['ROS_MASTER_URI'] for robot_name in BAXTER_ROBOTS]):
	import baxter_interface
	from baxter_interface import Gripper
else:
	raise EnvironmentError("You're not on either a Baxter or a Sawyer")

import threading

import os, sys
planningPath = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
sys.path.append(planningPath)
srvPath = os.path.join(planningPath, 'srv')
sys.path.append(srvPath)

from planning.srv import BuildStructure, BuildStructureResponse

# All in meters (m)
BLOCK_SIDE_LENGTH = 0.062
BLOCK_HEIGHT = 0.018
BLOCK_TOTAL_HEIGHT = 0.04
GRIPPER_LENGTH = 0.15


def inflate_gripper(gripper, lock, timeLimit = float('inf')):
    try:
        lock.acquire()
        DELAY = 1
        count = 0
        while lock.locked() and count * DELAY < timeLimit:
            gripper.inflate()
            rospy.sleep(DELAY)
            count += 1

        if count * DELAY >= timeLimit:
            lock.release()

        print('Exiting thread')
        sys.stdout.flush()
    except (KeyboardInterrupt, SystemExit):
        print('Forcefully exiting the inflate thread')
        sys.stdout.flush()
        lock.release()
        gripper.deflate()
        rospy.sleep(1)

def stop_inflating_gripper(gripper, lock):
    lock.release()
    print('Released lock')
    sys.stdout.flush()

def deflate_gripper(gripper, time=2):
    gripper.deflate()
    rospy.sleep(time)
    print('Done deflating')
    sys.stdout.flush()
    

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


def move_arm_to(planner, x, y, z, ox, oy, oz, ow, orientation_constraint=None):
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
            if orientation_constraint:
                plan = planner.plan_to_pose(pick, [orientation_constraint])
            else:
                plan = planner.plan_to_pose(pick, list())

            # if not cont.execute_path(plan);
            if not planner.execute_plan(plan):
                raise Exception()
        except Exception as e:
            print("Exception has occured", str(e))
            continue
        else:
            break

def buildStructureMain(blocks_flattened, width, height):
    blocks = np.reshape(blocks_flattened, (width, height))

    # Return output of service
    errorCode = 0
    return BuildStructureResponse(errorCode)
# """
# Inputs:
# blockPositions : int8[][] (the input given by planning/srv/BuildStructure.srv)

# Outputs:
# errorCode : int8 (the output required by planning/srv/BuildStructure.srv, 0 = successful run)
# """

# planner = PathPlanner("right_arm")

# # Set up the right gripper
# gripper = Gripper('right')
# gripper.inflate = lambda: gripper.close()
# gripper.deflate = lambda: gripper.open()

# # Kp = 0.1 * np.array([0.3, 2, 1, 1.5, 2, 2, 3]) # Stolen from 106B Students
# # Kd = 0.01 * np.array([2, 1, 2, 0.5, 0.5, 0.5, 0.5]) # Stolen from 106B Students
# # Ki = 0.01 * np.array([1, 1, 1, 1, 1, 1, 1]) # Untuned
# # Kw = np.array([0.9, 0.9, 0.9, 0.9, 0.9, 0.9, 0.9]) # Untuned

# # Create a path constraint for the arm
# orientation_constraint = OrientationConstraint()
# orientation_constraint.link_name = "right"
# orientation_constraint.header.frame_id = "base"
# orientation_constraint.orientation.y = -1.0
# orientation_constraint.absolute_x_axis_tolerance = 0.1
# orientation_constraint.absolute_y_axis_tolerance = 0.1
# orientation_constraint.absolute_z_axis_tolerance = 0.1
# orientation_constraint.weight = 1.0

# # Surrounding objects for Sawyer to plan around
# # add_block_obstacle(planner, [0.303, 0.79, 0.67], 'left_wall', [10, 10, 0], [0.56, 0.0, 0.0, 1.0])
# # add_block_obstacle(planner, [0,0,0], 'ceiling', [10, 5, 1])
# # add_block_obstacle(planner, [0.963, -0.15, -0.1], 'table', [10, 10, 0])
# # add_block_obstacle(planner, [0.81, -0.26, -0.1 + (0.5 * BLOCK_TOTAL_HEIGHT)], 'baseplate', [5 * BLOCK_SIDE_LENGTH, 6 * BLOCK_SIDE_LENGTH , 0.5 * BLOCK_TOTAL_HEIGHT])

# # Instantiate the threading Lock to control inflation/deflation
# inflationLock = threading.Lock()

# top_left_x = 0.93
# top_left_y = -0.08

# # start_x, start_y, start_z = 0.662, 0.039, 0.1
# start_x, start_y, start_z = 0.828, -0.258, 0.102
# block_x, block_y, block_z = 0.820, -0.246, 0.004

# def pick_and_place_block(dest_x, dest_y, dest_z, block_name):

#     raw_input("Press <Enter> to move the arm to x and y of pick up block: ")
#     move_arm_to(planner, start_x, start_y, start_z, 0, -1, 0, 0)

#     raw_input("Press <Enter> to move the arm to pick up the block: ")
#     move_arm_to(planner, block_x, block_y, block_z, 0, -1, 0, 0, orientation_constraint)
#     #move_arm_to(planner, start_x, start_y, -0.07, 0, -1, 0, 0, orientation_constraint)
    
#     inflateThread = threading.Thread(target=inflate_gripper, args=(gripper, inflationLock))
#     inflateThread.daemon = True
#     inflateThread.start()

#     raw_input("Press <Enter> to move the arm to move up with the block: ")
#     move_arm_to(planner, start_x, start_y, start_z, 0, -1, 0, 0)

#     raw_input("Press <Enter> to move the arm to x and y destinations: ")
#     move_arm_to(planner, dest_x, dest_y, start_z, 0, -1, 0, 0)

#     raw_input("Press <Enter> to drop the block: ")
#     move_arm_to(planner, dest_x, dest_y, dest_z, 0, -1, 0, 0, orientation_constraint)

#     stop_inflating_gripper(gripper, inflationLock)
#     inflateThread.join()  # Waits for thread to close
#     deflate_gripper(gripper)

#     raw_input("Press <Enter> to come up: ")
#     move_arm_to(planner, dest_x, dest_y, start_z, 0, -1, 0, 0)

#     raw_input("Press <Enter> to move to starting position: ")
#     move_arm_to(planner, start_x, start_y, start_z, 0, -1, 0, 0)

#     # add_block_obstacle(planner, [dest_x, dest_y, dest_z], block_name, [0.5 * BLOCK_SIDE_LENGTH, 0.5 * BLOCK_SIDE_LENGTH , 0.5 * BLOCK_TOTAL_HEIGHT])


# #pick_and_place_block(0.81, -0.27, -0.07, 'red_block')
# pick_and_place_block(0.73, 0.127, 0.002, 'red_block')

# # raw_input("Press <Enter> to place next block: ")

# # pick_and_place_block(0.86, -0.27, -0.07, 'yellow_block')

# # raw_input("Press <Enter> to place next block: ")

# # pick_and_place_block(0.83, -0.2, -0.07, 'next_block')

# raw_input("Press <Enter> to terminate: ")
# print("Done building structure!")

def initialize_service():
	rospy.init_node('build_structure_node')
	service = rospy.Service('build_structure', BuildStructure, buildStructureMain)
	print('Ready to build structures')
	rospy.spin()

if __name__ == '__main__':
    initialize_service()