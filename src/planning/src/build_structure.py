# #!/usr/bin/env python
# """
# Path Planning Script for Creator
# Authors: Aakarsh Gupta, Amay Saxena, Sairanjith Thalanki
# """

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
    import intera_interface
    from intera_interface import Gripper
	# raise EnvironmentError("You're not on either a Baxter or a Sawyer")

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


# def inflate_gripper(gripper, lock, timeLimit = float('inf')):
#     try:
#         lock.acquire()
#         DELAY = 1
#         count = 0
#         while lock.locked() and count * DELAY < timeLimit:
#             gripper.inflate()
#             rospy.sleep(DELAY)
#             count += 1

#         if count * DELAY >= timeLimit:
#             lock.release()

#         print('Exiting thread')
#         sys.stdout.flush()
#     except (KeyboardInterrupt, SystemExit):
#         print('Forcefully exiting the inflate thread')
#         sys.stdout.flush()
#         lock.release()
#         gripper.deflate()
#         rospy.sleep(1)

# def stop_inflating_gripper(gripper, lock):
#     lock.release()
#     print('Released lock')
#     sys.stdout.flush()

# def deflate_gripper(gripper, time=2):
#     gripper.deflate()
#     rospy.sleep(time)
#     print('Done deflating')
#     sys.stdout.flush()
    

# def add_block_obstacle(planner, position, name, 
#     size=[BLOCK_SIDE_LENGTH, BLOCK_SIDE_LENGTH, BLOCK_TOTAL_HEIGHT], orient = [0.0, 0.0, 0.0, 1.0]):

#     obstacle = PoseStamped()
#     obstacle.pose.position.x = position[0]
#     obstacle.pose.position.y = position[1]
#     obstacle.pose.position.z = position[2]

#     obstacle.pose.orientation.x = orient[0]
#     obstacle.pose.orientation.y = orient[1]
#     obstacle.pose.orientation.z = orient[2]
#     obstacle.pose.orientation.w = orient[3]

#     obstacle.header.frame_id = "base"
#     planner.add_box_obstacle(size, name, obstacle)


# def move_arm_to(planner, x, y, z, ox, oy, oz, ow, orientation_constraint=None):
#     while not rospy.is_shutdown():
#         try:
#             pick = PoseStamped()
#             pick.header.frame_id = "base"

#             #x, y, and z position
#             pick.pose.position.x = x
#             pick.pose.position.y = y
#             pick.pose.position.z = z

#             #Orientation as a quaternion
#             pick.pose.orientation.x = ox
#             pick.pose.orientation.y = oy
#             pick.pose.orientation.z = oz
#             pick.pose.orientation.w = ow

#             # cont = controller.Controller(Kp, Kd, Ki, Kw, Limb('right_arm'))
#             if orientation_constraint:
#                 plan = planner.plan_to_pose(pick, [orientation_constraint])
#             else:
#                 plan = planner.plan_to_pose(pick, list())

#             # if not cont.execute_path(plan);
#             if not planner.execute_plan(plan):
#                 raise Exception()
#         except Exception as e:
#             print("Exception has occured", str(e))
#             continue
#         else:
#             break

# def buildStructureMain(inputs):
#     blocks, num_layers, width, height = inputs.blocks, inputs.num_layers, inputs.width, inputs.height
#     blocks = np.reshape(blocks, (num_layers, width, height))

#     return buildStructureHelper(blocks)

# def buildStructureHelper(blocks):
#     # Return output of service
#     errorCode = 0
#     return BuildStructureResponse(errorCode)
# """
# Inputs:
# blockPositions : int8[][] (the input given by planning/srv/BuildStructure.srv)

# Outputs:
# errorCode : int8 (the output required by planning/srv/BuildStructure.srv, 0 = successful run)
# """

# # planner = PathPlanner("right_arm")

# # # Set up the right gripper
# # gripper = Gripper('right')
# # gripper.inflate = lambda: gripper.close()
# # gripper.deflate = lambda: gripper.open()

# # # Kp = 0.1 * np.array([0.3, 2, 1, 1.5, 2, 2, 3]) # Stolen from 106B Students
# # # Kd = 0.01 * np.array([2, 1, 2, 0.5, 0.5, 0.5, 0.5]) # Stolen from 106B Students
# # # Ki = 0.01 * np.array([1, 1, 1, 1, 1, 1, 1]) # Untuned
# # # Kw = np.array([0.9, 0.9, 0.9, 0.9, 0.9, 0.9, 0.9]) # Untuned

# # # Create a path constraint for the arm
# # orientation_constraint = OrientationConstraint()
# # orientation_constraint.link_name = "right"
# # orientation_constraint.header.frame_id = "base"
# # orientation_constraint.orientation.y = -1.0
# # orientation_constraint.absolute_x_axis_tolerance = 0.1
# # orientation_constraint.absolute_y_axis_tolerance = 0.1
# # orientation_constraint.absolute_z_axis_tolerance = 0.1
# # orientation_constraint.weight = 1.0

# # # Surrounding objects for Sawyer to plan around
# # # add_block_obstacle(planner, [0.303, 0.79, 0.67], 'left_wall', [10, 10, 0], [0.56, 0.0, 0.0, 1.0])
# # # add_block_obstacle(planner, [0,0,0], 'ceiling', [10, 5, 1])
# # # add_block_obstacle(planner, [0.963, -0.15, -0.1], 'table', [10, 10, 0])
# # # add_block_obstacle(planner, [0.81, -0.26, -0.1 + (0.5 * BLOCK_TOTAL_HEIGHT)], 'baseplate', [5 * BLOCK_SIDE_LENGTH, 6 * BLOCK_SIDE_LENGTH , 0.5 * BLOCK_TOTAL_HEIGHT])

# # # Instantiate the threading Lock to control inflation/deflation
# # inflationLock = threading.Lock()

# # top_left_x = 0.93
# # top_left_y = -0.08

# # # start_x, start_y, start_z = 0.662, 0.039, 0.1
# # start_x, start_y, start_z = 0.828, -0.258, 0.102
# # block_x, block_y, block_z = 0.820, -0.246, 0.004

# # def pick_and_place_block(dest_x, dest_y, dest_z, block_name):

# #     raw_input("Press <Enter> to move the arm to x and y of pick up block: ")
# #     move_arm_to(planner, start_x, start_y, start_z, 0, -1, 0, 0)

# #     raw_input("Press <Enter> to move the arm to pick up the block: ")
# #     move_arm_to(planner, block_x, block_y, block_z, 0, -1, 0, 0, orientation_constraint)
# #     #move_arm_to(planner, start_x, start_y, -0.07, 0, -1, 0, 0, orientation_constraint)
    
# #     inflateThread = threading.Thread(target=inflate_gripper, args=(gripper, inflationLock))
# #     inflateThread.daemon = True
# #     inflateThread.start()

# #     raw_input("Press <Enter> to move the arm to move up with the block: ")
# #     move_arm_to(planner, start_x, start_y, start_z, 0, -1, 0, 0)

# #     raw_input("Press <Enter> to move the arm to x and y destinations: ")
# #     move_arm_to(planner, dest_x, dest_y, start_z, 0, -1, 0, 0)

# #     raw_input("Press <Enter> to drop the block: ")
# #     move_arm_to(planner, dest_x, dest_y, dest_z, 0, -1, 0, 0, orientation_constraint)

# #     stop_inflating_gripper(gripper, inflationLock)
# #     inflateThread.join()  # Waits for thread to close
# #     deflate_gripper(gripper)

# #     raw_input("Press <Enter> to come up: ")
# #     move_arm_to(planner, dest_x, dest_y, start_z, 0, -1, 0, 0)

# #     raw_input("Press <Enter> to move to starting position: ")
# #     move_arm_to(planner, start_x, start_y, start_z, 0, -1, 0, 0)

# #     # add_block_obstacle(planner, [dest_x, dest_y, dest_z], block_name, [0.5 * BLOCK_SIDE_LENGTH, 0.5 * BLOCK_SIDE_LENGTH , 0.5 * BLOCK_TOTAL_HEIGHT])


# # #pick_and_place_block(0.81, -0.27, -0.07, 'red_block')
# # pick_and_place_block(0.73, 0.127, 0.002, 'red_block')

# # # raw_input("Press <Enter> to place next block: ")

# # # pick_and_place_block(0.86, -0.27, -0.07, 'yellow_block')

# # # raw_input("Press <Enter> to place next block: ")

# # # pick_and_place_block(0.83, -0.2, -0.07, 'next_block')

# # raw_input("Press <Enter> to terminate: ")
# # print("Done building structure!")

# def initialize_service():
# 	rospy.init_node('build_structure_node')
# 	service = rospy.Service('build_structure', BuildStructure, buildStructureMain)
# 	print('Ready to build structures')
# 	rospy.spin()

# if __name__ == '__main__':
#     # initialize_service()

#     import numpy as np
#     workspace_src_path = os.path.abspath(os.path.join(planningPath, '..'))
#     creator_cv_path = os.path.join(workspace_src_path, 'creator_cv')
#     creator_cv_src_path = os.path.join(creator_cv_path, 'src')
#     creator_cv_blocks_path = os.path.join(creator_cv_src_path, 'blocks')
#     file_name = 'test.npy'
#     blocks = np.load(creator_cv_blocks_path + '/' + file_name)
#     print(blocks)
#     buildStructureHelper(blocks)



#!/usr/bin/env python

"""
Path Planning Script for Creator
"""



import rospy
import numpy as np
import controller
import ar 

from moveit_msgs.msg import OrientationConstraint
from geometry_msgs.msg import PoseStamped

from path_planner import PathPlanner
# import arduino_communication as ard_com
import intera_interface

top_left_x = 0
top_left_y = 0
top_left_z = 0

_EPS = np.finfo(float).eps * 4.0

class Robot:
    def __init__(self, path_planner, gripper, gripper_name, gripper_length):
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
        # self.grab = grab
        # self.release = release

BLOCK_SIDE_LENGTH = 0.0635
HALF_BLOCK_SIDE_LENGTH = BLOCK_SIDE_LENGTH / 2.0

BLOCK_SQUARE_HEIGHT = 0.038
BLOCK_TOTAL_HEIGHT = 0.059
BLOCK_STUD_HEIGHT = BLOCK_TOTAL_HEIGHT - BLOCK_SQUARE_HEIGHT

BLOCK_PICK_BUFFER = 0.005


# Helper functions
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
        Takes the ROW and COLUMN stud numbers ([0-11], [0-9]) at which the block is to be placed.
        Calculates dest_x, dest_y that Sawyer needs to move to.
    '''

    global top_left_x
    global top_left_y

    top_left_stud_x = block_coord_tl[1]
    top_left_stud_y = block_coord_tl[0]

    dest_x = top_left_x - (top_left_stud_x * HALF_BLOCK_SIDE_LENGTH) - 0.065
    dest_y = top_left_y - (top_left_stud_y * HALF_BLOCK_SIDE_LENGTH) - 0.065

    return [dest_x, dest_y]

def start_num_to_sawyer(start_block_number):
    '''
        Takes one of the starting block bin numbers [0-7] from which the block is to be picked up.
        Calculates dest_x, dest_y that Sawyer needs to move to in order to pick the block up.
        7 3
        6 2
        5 1
        4 0
    '''
    START_BLOCK_SPACING = 0.028

    # ar_tag_x = top_left_x + 0.061
    # ar_tag_y = top_left_y + 0.063

    # print('Top left:', top_left_x, top_left_y)
    # print('AR tag:', ar_tag_x, ar_tag_y)

    # if 0 <= start_block_number < 4:
    #     start_x = top_left_x - 0.076
    # else:
    #     start_x = top_left_x - 0.076 - START_BLOCK_SPACING - BLOCK_SIDE_LENGTH

    # start_y = top_left_y - 0.037 - (3 - start_block_number % 4) * (BLOCK_SIDE_LENGTH + START_BLOCK_SPACING)


    '''
    Coordinate system for below function:
      ^ +x
      | 
    <-+y

    '''

    BUFFER_X = -0.009 + 0.0025
    BUFFER_Y = -0.003 + 0.0025

    # MULTIPLIER_BUFFER_Y = -0.005
    # MULTIPLIER_BUFFER_X = -0.0125 / 3
    MULTIPLIER_BUFFER_X = 0
    MULTIPLIER_BUFFER_Y = 0

    if 0 <= start_block_number < 4:
        start_y = top_left_y + 0.076 + BUFFER_Y
    else:
        start_y = top_left_y + (0.076 + START_BLOCK_SPACING + BLOCK_SIDE_LENGTH) + BUFFER_Y + MULTIPLIER_BUFFER_Y

    start_x = top_left_x - 0.037 - (3 - start_block_number % 4) * (BLOCK_SIDE_LENGTH + START_BLOCK_SPACING - MULTIPLIER_BUFFER_X) + BUFFER_X

    # if start_block_number % 4 == 3:
    #     start_y = top_left_y - 0.037
    # elif start_block_number % 4 == 2:
    #     start_y = top_left_y - 0.037 - 1 * (BLOCK_SIDE_LENGTH - START_BLOCK_SPACING)
    # elif start_block_number % 4 == 1:
    #     start_y = top_left_y - 0.037 - 2 * (BLOCK_SIDE_LENGTH - START_BLOCK_SPACING)
    # else:
    #     start_y = top_left_y - 0.037 - 3 * (BLOCK_SIDE_LENGTH - START_BLOCK_SPACING)

    # if start_block_number in range(4):
    #     dest_x = top_left_x + 0.076
    # else:
    #     dest_x = top_left_x + (2 * 0.076)

    # if start_block_number in [0, 4]:
    #     dest_y = top_left_y 
    # elif start_block_number in [1, 5]:
    #     dest_y = top_left_y
    # elif start_block_number in [2, 6]:
    #     dest_y = top_left_y
    # else:
    #     dest_y = top_left_y

    return [start_x, start_y]

def move_arm_to(robot, x,y,z, ox, oy, oz, ow, orient_constraint=True):
    while not rospy.is_shutdown():
        try:
            print("Creating pick")
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
            print('Checking orientation constraint')
            if orient_constraint:
                plan = robot.planner.plan_to_pose(pick, [robot.orien_const])
            else:
                plan = robot.planner.plan_to_pose(pick, list())

            # if not cont.execute_path(plan);
            if not robot.planner.execute_plan(plan):
                raise Exception()
        except Exception as e:
            print("Exception has occured")
            print(e)
            continue
        else:
            break

def pick_and_place_block(robot, start_x, start_y, start_z, dest_x, dest_y, dest_z, block_name):

    z_offset = 0.1
    
    raw_input("Press <Enter> to move the arm over the block to be picked up: ")
    move_arm_to(robot, start_x, start_y, start_z + z_offset, 1, 0, 0, 0, False)

    print("hello")
    # raw_input("Press <Enter> to move the arm to pick up the block: ")
    # move_arm_to(robot, start_x, start_y, start_z + 0.01, 1, 0, 0, 0, False)    

    raw_input("Press <Enter> to move the arm to pick up the block: ")
    move_arm_to(robot, start_x, start_y, start_z, 1, 0, 0, 0, False)    
    
    # ard_com.inflate()
    
    raw_input("Press <Enter> to lift block: ")
    move_arm_to(robot, start_x, start_y, start_z + z_offset, 1, 0, 0, 0, False)

    raw_input("Press <Enter> to move the arm to x and y destinations: ")
    move_arm_to(robot, dest_x, dest_y - 0.03, dest_z + z_offset, 1, 0, 0, 0, False)

    raw_input("Press <Enter> to move the arm stabilize x and y destinations: ")
    move_arm_to(robot, dest_x, dest_y - 0.03, dest_z + z_offset, 1, 0, 0, 0, False)

    raw_input("Press <Enter> to place the block: ")
    move_arm_to(robot, dest_x, dest_y, dest_z, 1, 0, 0, 0, False)

    # ard_com.deflate()

    raw_input("Press <Enter> to come up: ")
    move_arm_to(robot, dest_x, dest_y, dest_z + z_offset, 1, 0, 0, 0, False)

    raw_input("Press <Enter> to move to starting position: ")
    move_arm_to(robot, start_x, start_y, start_z + z_offset, 1, 0, 0, 0, False)

    add_block_obstacle(robot, [dest_x, dest_y, dest_z - (0.5 * BLOCK_SQUARE_HEIGHT)], block_name, [BLOCK_SIDE_LENGTH * 0.8, BLOCK_SIDE_LENGTH * 0.8, BLOCK_SQUARE_HEIGHT])
    
    print("Placed block!")

from ar_track_alvar_msgs.msg import AlvarMarkers
import math
import numpy as np
import rospy 

received_marker = False

marker_rotations = []
marker_translations = []

def ar_marker_callback(ar_pose):
    global received_marker
    if received_marker is False:
        print("Hit the ar_marker callback!")
        markers = ar_pose.markers
        assert len(markers) == 1
        marker = markers[0]
        pose = marker.pose.pose
        trans = np.array([pose.position.x, pose.position.y, pose.position.z])
        q = np.array([pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z])
        R = quaternion_matrix(q)
        marker_rotations.append(R)
        marker_translations.append(trans)
        received_marker = True
    else:
        pass

def transform(pc, R, t):
    return np.dot(R.T, ((pc - t).T)).T

def quaternion_matrix(quaternion):
    q = np.array(quaternion, dtype=np.float64, copy=True)
    n = np.dot(q, q)
    if n < _EPS:
        return numpy.identity(4)
    q *= math.sqrt(2.0 / n)
    q = np.outer(q, q)
    
    return np.array([
        [1.0-q[2, 2]-q[3, 3],     q[1, 2]-q[3, 0],     q[1, 3]+q[2, 0], 0.0],
        [    q[1, 2]+q[3, 0], 1.0-q[1, 1]-q[3, 3],     q[2, 3]-q[1, 0], 0.0],
        [    q[1, 3]-q[2, 0],     q[2, 3]+q[1, 0], 1.0-q[1, 1]-q[2, 2], 0.0],
        [                0.0,                 0.0,                 0.0, 1.0]])

def get_top_left_AR_tag():

    global received_marker
    ar_topic = '/ar_pose_marker'

    while not rospy.core.is_shutdown() and not received_marker:
        rospy.Subscriber(ar_topic, AlvarMarkers, ar_marker_callback)

    rospy.rostime.wallsleep(0.05)
    print("Got Marker!")

def main():
    # print("Here are the marker rotations: " + str(marker_rotations))
    # print("Here are the marker translations: " + str(marker_translations))

    #assert len(marker_translations) == 1
    #R = marker_rotations[0]
    #t = marker_translations[0]

    global top_left_x
    global top_left_y
    global top_left_z

    # get_top_left_AR_tag()
    top_left_x, top_left_y, top_left_z = 0.969, 0.143, -0.214
    # assert len(marker_translations) == 1
    # top_left_x, top_left_y, top_left_z = marker_translations[0]

    sawyer = Robot(PathPlanner("right_arm"), None, "right_gripper", 0.095)

    # Adding environment obstacles for the robot to avoid during path planning
    # add_block_obstacle(sawyer, position=[0.67452, -0.4455, 0.916557], name='right_wall', size=[5, 0, 5], orient=[1, 0, 0, 0])
    # add_block_obstacle(sawyer, position=[0.460625, 0.5444, 0.916557], name='left_wall', size=[5, 0, 5], orient=[1, 0, 0, 0])
    # add_block_obstacle(sawyer, position=[, , TABLE_HEIGHT - sawyer.gripper_length], name='table', size=[3.5, 3.5, 0], orient=[0, 1, 0, 0])
    # add_block_obstacle(planner, [0,0,0], 'ceiling', [10, 5, 1])
    '''
    add_block_obstacle(sawyer, 
        position=[0.81, -0.26, -0.1 + (0.5 * BLOCK_TOTAL_HEIGHT)], 
        name='baseplate', 
        size=[5 * BLOCK_SIDE_LENGTH, 6 * BLOCK_SIDE_LENGTH , 0.5 * BLOCK_TOTAL_HEIGHT])

    add_block_obstacle(sawyer, 
        position=[0.81, -0.26, -0.1 - (BLOCK_TOTAL_HEIGHT / 2)], 
        name='table', 
        size=[5, 5, BLOCK_TOTAL_HEIGHT / 2])
    '''
    add_block_obstacle(sawyer, 
        position=[top_left_x, top_left_y, top_left_z - (BLOCK_TOTAL_HEIGHT / 2)], 
        name='table', 
        size=[5, 5, BLOCK_TOTAL_HEIGHT / 2])

    import random

    # BASE_PLATE_HEIGHT = 0.5 * BLOCK_SQUARE_HEIGHT
    # dest_x, dest_y = block_coord_to_sawyer((5, 4))
    # start_z_offset = 0.08
    # for layer in range(2):
    #     new_dest_z = top_left_z + BLOCK_SQUARE_HEIGHT + sawyer.gripper_length + start_z_offset + BASE_PLATE_HEIGHT + layer * BLOCK_SQUARE_HEIGHT
    #     add_block_obstacle(sawyer, [dest_x, dest_y, new_dest_z - (0.5 * BLOCK_SQUARE_HEIGHT)], 'ehllo' + str(random.randint(1, 1000)), [BLOCK_SIDE_LENGTH, BLOCK_SIDE_LENGTH, BLOCK_SQUARE_HEIGHT])
    # #TODO: Finish adding as many environment constraints as possible without compromising on path planning.
    # return

    dest_x, dest_y = block_coord_to_sawyer((5, 4))
    # add_block_obstacle(sawyer, position=[dest_x, dest_y, top_left_z], name='table', size=[5, 5, 0], orient=[1, 0, 0, 0])
    # add_block_obstacle(sawyer, position=[dest_x, dest_y, top_left_z - 0.006 + 0.25 * BLOCK_SQUARE_HEIGHT], name='baseplate', size=[5 * BLOCK_SIDE_LENGTH , 6 * BLOCK_SIDE_LENGTH, .5 * BLOCK_SQUARE_HEIGHT], orient=[1, 0, 0, 0])

    # print('Top left:', top_left_x, top_left_y)
    
    #start_x3 = top_left_x - 0.074 - (BLOCK_SIDE_LENGTH / 2.0)
    #start_y3 = top_left_y + 0.0425 + (BLOCK_SIDE_LENGTH / 2.0)
    #print(start_x3, start_y3)
    
    start_x3, start_y3 = start_num_to_sawyer(3)
    print(start_x3, start_y3)

    # print()
    # for i in range(8):
    #     print(start_num_to_sawyer(i))

    #while 1 < 2:
        # pick_and_place_block(sawyer, start_x3, start_y3, top_left_z + BLOCK_SQUARE_HEIGHT, dest_x, dest_y, top_left_z + (0.5 * BLOCK_SQUARE_HEIGHT), 'block_test') 
        #pick_and_place_block(sawyer, start_x3, start_y3, top_left_z + BLOCK_SQUARE_HEIGHT, dest_x, dest_y, top_left_z + BLOCK_SQUARE_HEIGHT, 'block_test') 
    #pick_and_place_block(sawyer, top_left_x, top_left_y, top_left_z, dest_x, dest_y, top_left_z + BLOCK_SQUARE_HEIGHT, 'block_test')  

    start_z_offset = 0.08
    BASE_PLATE_HEIGHT = 0.5 * BLOCK_SQUARE_HEIGHT
    print(top_left_x, top_left_y, top_left_z)

    # import random

    # for layer in range(3):
    #     # dest_x, dest_y = block_coord_to_sawyer((5 + layer * 2, 4))
    #     # new_dest_z = top_left_z + BLOCK_SQUARE_HEIGHT + sawyer.gripper_length + start_z_offset + BASE_PLATE_HEIGHT
    #     new_dest_z = top_left_z + BLOCK_SQUARE_HEIGHT + sawyer.gripper_length + start_z_offset + BASE_PLATE_HEIGHT + layer * BLOCK_SQUARE_HEIGHT
    #     pick_and_place_block(sawyer, 
    #         start_x3, start_y3, top_left_z + BLOCK_SQUARE_HEIGHT + sawyer.gripper_length + start_z_offset, 
    #         dest_x, dest_y, new_dest_z, 
    #         'block_' + str(random.randint(1, 1000)))

    import numpy as np
    workspace_src_path = os.path.abspath(os.path.join(planningPath, '..'))
    creator_cv_path = os.path.join(workspace_src_path, 'creator_cv')
    creator_cv_src_path = os.path.join(creator_cv_path, 'src')
    creator_cv_blocks_path = os.path.join(creator_cv_src_path, 'blocks')
    file_name = 'test.npy'
    blocks = np.load(creator_cv_blocks_path + '/' + file_name)

    for layer, layer_blocks = enumerate(blocks):
        for layer_block in layer_blocks:
            dest_x, dest_y = block_coord_to_sawyer(layer_block)
            # new_dest_z = top_left_z + BLOCK_SQUARE_HEIGHT + sawyer.gripper_length + start_z_offset + BASE_PLATE_HEIGHT
            new_dest_z = top_left_z + BLOCK_SQUARE_HEIGHT + sawyer.gripper_length + start_z_offset + BASE_PLATE_HEIGHT + layer * BLOCK_SQUARE_HEIGHT
            pick_and_place_block(sawyer, 
                start_x3, start_y3, top_left_z + BLOCK_SQUARE_HEIGHT + sawyer.gripper_length + start_z_offset, 
                dest_x, dest_y, new_dest_z, 
                'block_' + str(random.randint(1, 10000)))

    # layer = 1
    # new_dest_z = top_left_z + BLOCK_SQUARE_HEIGHT + sawyer.gripper_length + start_z_offset + BASE_PLATE_HEIGHT + layer * BLOCK_SQUARE_HEIGHT
    # pick_and_place_block(sawyer, 
    #     start_x3, start_y3, top_left_z + BLOCK_SQUARE_HEIGHT + sawyer.gripper_length + start_z_offset, 
    #     dest_x, dest_y, new_dest_z, 
    #     'block_' + str(random.randint(1, 1000)))
        # add_block_obstacle(sawyer, 
        #     position=[top_left_x, top_left_y, top_left_z - (BLOCK_TOTAL_HEIGHT / 2)], 
        #     name='block_' + str(random.randint(1, 1000)), 
        #     size=[BLOCK_SQUARE_HEIGHT, BLOCK_SQUARE_HEIGHT, BLOCK_TOTAL_HEIGHT])
    #move_arm_to(sawyer, top_left_x, top_left_y, top_left_z + 0.1 , 1, 0, 0, 0, False)

    #move_arm_to(sawyer, start_x3, start_y3, top_left_z + BLOCK_SQUARE_HEIGHT + 0.1, 1, 0, 0, 0, False)
    #move_arm_to(sawyer, 1,1,1, 1, 0, 0, 0, False)
    #move_arm_to(sawyer, start_x3, start_y3, top_left_z + 2 * BLOCK_SQUARE_HEIGHT, 1, 0, 0, 0, False)
    
    # Final Code to use.
    # i = 0
    # height_constant = 0.5
    # for layer in layers:
    #     for block in get_blocks_from_layer(layer):
    #         start_x, start_y = start_num_to_sawyer(i % 8)
    #         dest_x, dest_y = block_coord_to_sawyer(block)
    #         dest_z = top_left_z + (height_constant * BLOCK_SQUARE_HEIGHT)
    #         pick_and_place_block(sawyer, start_x, start_y, -0.0320774, dest_x, dest_y, dest_z, 'block_' + str(i))
    #         i += 1
    #     height_constant += 1
    # return 'The structure has been built!'

if __name__ == '__main__':
    """
    Steps:
    source devel/setup.bash; catkin_make; ./baxter.sh ada.local
    1) python run_camera.py within planning/src in Sawyer
    2) roslaunch ar_track_alvar sawyer_ar.launch in Sawyer
    3) rosrun intera_interface enable_robot.py -e in Sawyer
    3.5) rosrun intera_interface joint_trajectory_action_server.py in Sawyer
    4) roslaunch sawyer_moveit_config sawyer_moveit.launch electric_gripper:=false in Sawyer
    5) python path_test.py within planning/src in Sawyer

    """
    rospy.init_node('moveit_node')
    main()

    '''
    +ve y
    |
    |
    |-------- -ve x
    |
    |

    Ahad: The coordinate system I found using RViz flipped x and y in the above diagrams (it's also implied by the values passed into the boundary constraints above):
    +ve x
    |
    |
    |-------- -ve y
    |
    |
    '''
