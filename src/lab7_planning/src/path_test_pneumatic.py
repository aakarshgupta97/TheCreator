#!/usr/bin/env python

"""
Path Planning Script for Creator
"""

import sys
import rospy
import numpy as np
import controller

from moveit_msgs.msg import OrientationConstraint
from geometry_msgs.msg import PoseStamped

from path_planner import PathPlanner
import intera_interface
# import baxter_interface
import threading

class Robot:
	def __init__(self, path_planner, gripper, gripper_name, gripper_length, grab, release):
		self.planner = path_planner
		self.gripper = gripper

		# Create a path constraint for the arm
		self.orien_const = OrientationConstraint()
		self.orien_const.link_name = gripper_name
		self.orien_const.header.frame_id = "base"
		self.orien_const.orientation.y = -1.0
		self.orien_const.absolute_x_axis_tolerance = 0.1
		self.orien_const.absolute_y_axis_tolerance = 0.1
		self.orien_const.absolute_z_axis_tolerance = 0.1
		self.orien_const.weight = 1.0

		self.gripper_length = gripper_length
		self.grab = grab
		self.release = release

# For 2x2 big boys
BLOCK_SIDE_LENGTH = 0.0635
BLOCK_SQUARE_HEIGHT = 0.038
BLOCK_TOTAL_HEIGHT = 0.059
BLOCK_STUD_HEIGHT = BLOCK_TOTAL_HEIGHT - BLOCK_SQUARE_HEIGHT
BLOCK_PICK_BUFFER = 0.005

#frame is right_vacuum_gripper_base
TABLE_HEIGHT = -0.080
TOP_LEFT_X = .80
TOP_LEFT_Y = .11


# Helper functions
def inflate(robot, lock, timeLimit = float('inf')):
    try:
        lock.acquire()
        DELAY = 1
        count = 0
        while lock.locked() and count * DELAY < timeLimit:
            robot.grab()
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
        robot.release()
        rospy.sleep(1)

def deflate(robot, lock, time=2):
    lock.release()
    print('Released lock')
    sys.stdout.flush()
    robot.release()
    rospy.sleep(time)
    print('Done deflating')
    sys.stdout.flush()   

def add_block_obstacle(robot, position, name, size=[BLOCK_SIDE_LENGTH, BLOCK_SIDE_LENGTH, BLOCK_TOTAL_HEIGHT], orient = [0.0, 0.0, 0.0, 1.0]):
    obstacle = PoseStamped()
    obstacle.pose.position.x = position[0]
    obstacle.pose.position.y = position[1]
    obstacle.pose.position.z = position[2]

    obstacle.pose.orientation.x = orient[0]
    obstacle.pose.orientation.y = orient[1]
    obstacle.pose.orientation.z = orient[2]
    obstacle.pose.orientation.w = orient[3]

    obstacle.header.frame_id = "base"
    robot.planner.add_box_obstacle(size, name, obstacle)

def block_coord_to_sawyer(block_coord_tl):

    '''
	    Takes the row and column stud numbers ([0, 11], [0, 9]) at which the block is to be placed.
	    Calculates dest_x, dest_y that Sawyer needs to move to.
    '''
    HALF_BLOCK_SIDE_LENGTH = BLOCK_SIDE_LENGTH / 2.0

    top_left_stud_x = block_coord_tl[0]
    top_left_stud_y = block_coord_tl[1]

    dest_x = top_left_center_x - (top_left_stud_x * HALF_BLOCK_SIDE_LENGTH)
    dest_y = top_left_center_y - (top_left_stud_y * HALF_BLOCK_SIDE_LENGTH)

    return [dest_x, dest_y]

def move_arm_to(robot, x,y,z, ox, oy, oz, ow, orient_constraint=True):
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
                plan = robot.planner.plan_to_pose(pick, [orien_const])
            else:
                plan = robot.planner.plan_to_pose(pick, list())

            # if not cont.execute_path(plan);
            if not robot.planner.execute_plan(plan):
                raise Exception()
        except Exception as e:
            print("Exception has occured")
            continue
        else:
            break

def pick_and_place_block(robot, inflationLock, start_x, start_y, start_z, dest_x, dest_y, dest_z, block_name):
    
    raw_input("Press <Enter> to move the arm to over the block to be picked up: ")
    move_arm_to(robot, start_x, start_y, start_z, 0, -1, 0, 0, False)

    raw_input("Press <Enter> to move the arm to pick up the block: ")
    move_arm_to(robot, start_x, start_y, -0.07, 0, -1, 0, 0)#

    #inflateThread = threading.Thread(target=inflate, args=(robot, inflationLock))
    #inflateThread.daemon = True
    #inflateThread.start()
    
    raw_input("Press <Enter> to lift block: ")
    move_arm_to(robot, start_x, start_y, start_z, 0, -1, 0, 0, False)

    raw_input("Press <Enter> to move the arm to x and y destinations: ")
    move_arm_to(robot, dest_x, dest_y, 0.1, 0, -1, 0, 0, False)

    raw_input("Press <Enter> to place the block: ")
    move_arm_to(robot, dest_x, dest_y, 0.02 + dest_z, 0, -1, 0, 0) # Add _______ to avoid collision. Let gravity do the rest.

    #stop_inflate(robot, inflationLock)
    #inflateThread.join()  # Waits for thread to close
    #deflate(robot)

    raw_input("Press <Enter> to come up: ")
    move_arm_to(robot, dest_x, dest_y, 0.1, 0, -1, 0, 0, False)

    raw_input("Press <Enter> to move to starting position: ")
    move_arm_to(robot, start_x, start_y, start_z, 0, -1, 0, 0, False)

    add_block_obstacle(robot, [dest_x, dest_y, dest_z], block_name, [0.5 * BLOCK_SIDE_LENGTH, 0.5 * BLOCK_SIDE_LENGTH , 0.5 * BLOCK_TOTAL_HEIGHT])
    
    print("Placed block!")

def main():
	sawyer = Robot(PathPlanner("right_arm"), intera_interface.Gripper('left_gripper'), "right_gripper", 0.17, None, None)
	# sawyer = Robot(PathPlanner("right_arm"), "No", "right_gripper", 0.17, None, None)

	# Kp = 0.1 * np.array([0.3, 2, 1, 1.5, 2, 2, 3])
	# Kd = 0.01 * np.array([2, 1, 2, 0.5, 0.5, 0.5, 0.5])
	# Ki = 0.01 * np.array([1, 1, 1, 1, 1, 1, 1])
	# Kw = np.array([0.9, 0.9, 0.9, 0.9, 0.9, 0.9, 0.9])

	# Surrounding objects for Sawyer to plan around
	add_block_obstacle(sawyer, position=[0.57, 0.75, 0.04], name='left_wall', size=[3.5, 0, 3.5], orient=[1, 0, 0, 0])
	add_block_obstacle(sawyer, position=[0.60, 0.42, 1], name='false_ceiling', size=[3.5, 3.5, 0], orient=[0, 1, 0, 0])
	add_block_obstacle(sawyer, position=[0.52, 0.53, TABLE_HEIGHT - sawyer.gripper_length], name='table', size=[3.5, 3.5, 0], orient=[0, 1, 0, 0])
	add_block_obstacle(sawyer, position=[-0.40, 0.57, 0.8], name='back_wall', size=[0, 3.5, 3.5], orient=[0, 1, 0, 0])
	add_block_obstacle(sawyer, position=[0.59, -0.54, 0.34], name='false_right_wall', size=[3.5, 0, 3.5], orient=[0, 1, 0, 0])
	add_block_obstacle(sawyer, position=[0.70, 0.04, TABLE_HEIGHT - sawyer.gripper_length + (0.25 * BLOCK_SQUARE_HEIGHT)], name='baseplate', size=[5 * BLOCK_SIDE_LENGTH, 6 * BLOCK_SIDE_LENGTH , 0.5 * BLOCK_SQUARE_HEIGHT])

	# Instantiate the threading Lock to control inflation/deflation
	inflationLock = threading.Lock()

	start1_x, start1_y = 0.56, 0.25
	start_z = TABLE_HEIGHT - sawyer.gripper_length + BLOCK_PICK_BUFFER + BLOCK_SQUARE_HEIGHT
	baseplate_z = TABLE_HEIGHT - sawyer.gripper_length + BLOCK_PICK_BUFFER + .5 * BLOCK_SQUARE_HEIGHT
	pick_and_place_block(sawyer, inflationLock, start1_x, start1_y, start_z, TOP_LEFT_X, TOP_LEFT_Y, baseplate_z, 'red_block')    

	# for i in range(8)
	'''
    pick_and_place_block(sawyer, start_x, start_y, start_z, 0.81, -0.27, -0.07, 'red_block')

    raw_input("Press <Enter> to place next block: ")

    pick_and_place_block(sawyer, start_x, start_y, start_z, 0.86, -0.27, -0.07, 'yellow_block')

    raw_input("Press <Enter> to place next block: ")

    pick_and_place_block(sawyer, start_x, start_y, start_z, 0.83, -0.2, -0.07, 'next_block')

    raw_input("Press <Enter> to terminate: ")

    print("Done!")

	'''
if __name__ == '__main__':
    rospy.init_node('moveit_node')
    main()
