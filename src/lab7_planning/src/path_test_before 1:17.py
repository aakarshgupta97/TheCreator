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
#import arduino_communication as ard_com
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
	    Takes the COLUMN and ROW stud numbers ([0-9], [0-11]) at which the block is to be placed.
	    Calculates dest_x, dest_y that Sawyer needs to move to.
    '''

    global top_left_x
    global top_left_y

    top_left_stud_x = block_coord_tl[0]
    top_left_stud_y = block_coord_tl[1]

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

    BUFFER_X = -0.007
    BUFFER_Y = -0.007

    MULTIPLIER_BUFFER_Y = -0.005
    MULTIPLIER_BUFFER_X = -0.0125 / 3

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
            print(e)
            continue
        else:
            break

def pick_and_place_block(robot, start_x, start_y, start_z, dest_x, dest_y, dest_z, block_name):

    z_offset = 0.1
    
    raw_input("Press <Enter> to move the arm over the block to be picked up: ")
    move_arm_to(robot, start_x, start_y, start_z + z_offset, 1, 0, 0, 0, False)

    print("hello")
    raw_input("Press <Enter> to move the arm to pick up the block: ")
    move_arm_to(robot, start_x, start_y, start_z, 1, 0, 0, 0, False)    
    
    #ard_com.inflate()
    
    raw_input("Press <Enter> to lift block: ")
    move_arm_to(robot, start_x, start_y, start_z + z_offset, 1, 0, 0, 0, False)

    raw_input("Press <Enter> to move the arm to x and y destinations: ")
    move_arm_to(robot, dest_x, dest_y, dest_z + z_offset, 1, 0, 0, 0, False)

    raw_input("Press <Enter> to place the block: ")
    move_arm_to(robot, dest_x, dest_y, dest_z, 1, 0, 0, 0, False)

    #ard_com.deflate()

    raw_input("Press <Enter> to come up: ")
    move_arm_to(robot, dest_x, dest_y, dest_z + z_offset, 1, 0, 0, 0, False)

    raw_input("Press <Enter> to move to starting position: ")
    move_arm_to(robot, start_x, start_y, start_z + z_offset, 1, 0, 0, 0, False)

    # add_block_obstacle(robot, [dest_x, dest_y, dest_z + (0.5 * BLOCK_TOTAL_HEIGHT)], block_name, [0.5 * BLOCK_SIDE_LENGTH, 0.5 * BLOCK_SIDE_LENGTH , 0.5 * BLOCK_TOTAL_HEIGHT])
    
    print("Placed block!")
   

def main():
    # print("Here are the marker rotations: " + str(marker_rotations))
    # print("Here are the marker translations: " + str(marker_translations))

    #assert len(marker_translations) == 1
    #R = marker_rotations[0]
    #t = marker_translations[0]

    global top_left_x
    global top_left_y
    global top_left_z

    top_left_x, top_left_y, top_left_z = .976, 0.326, -0.059

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
    #TODO: Finish adding as many environment constraints as possible without compromising on path planning.


    dest_x, dest_y = block_coord_to_sawyer((4, 5))
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

    pick_and_place_block(sawyer, start_x3, start_y3, top_left_z + BLOCK_SQUARE_HEIGHT, dest_x, dest_y, top_left_z + BLOCK_SQUARE_HEIGHT, 'block_test')  
    #move_arm_to(sawyer, top_left_x, top_left_y, top_left_z + 0.1 , 1, 0, 0, 0, False)

    #move_arm_to(sawyer, start_x3, start_y3, top_left_z + BLOCK_SQUARE_HEIGHT + 0.1, 1, 0, 0, 0, False)
    #move_arm_to(sawyer, 1,1,1, 1, 0, 0, 0, False)
    #move_arm_to(sawyer, start_x3, start_y3, top_left_z + 2 * BLOCK_SQUARE_HEIGHT, 1, 0, 0, 0, False)
    # Final Code to use.
    # i = 0
    # for layer in layers:
    #     for block in get_blocks_from_layer(layer):
    #         start_x, start_y = start_num_to_sawyer(i % 8)
    #         dest_x, dest_y = block_coord_to_sawyer(block) #TODO: MAKE SURE IT'S COLUMN AND THEN ROW!
    #         pick_and_place_block(sawyer, start_x, start_y, -0.0320774, dest_x, dest_y, top_left_z + (0.5 * BLOCK_SQUARE_HEIGHT), 'block_' + str(start_num))
    #         i += 1

    # return 'The structure has been built!'

if __name__ == '__main__':
    """
    Steps:
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