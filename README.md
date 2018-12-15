# TheCreator
TheCreator is a student group project to program a robot to inspect a structure built out of MEGABLOKS and autonomously replicate the structure. We accomplished this task by using computer vision to generate a computerized model of the structure, which we then sent to a Sawyer robotic arm to pick and place blocks in their respective positions. 

After inspecting the structure from various angles, we will first need to create a model showing the composition of blocks in the structure. Sawyer will then pick up pieces from a table and place the pieces on a baseplate that is fixed to a table. It will continue to do this, building from the bottom of the structure up to the top, until it has replicated the original structure. The robot should be able to optimize its technique for placing the pieces based on the surroundings of a piece, such as if the piece is on an overhang or inside some other structure. 

## Dependencies
TheCreator is designed for ROS Indigo, Python 3, and other ROS module dependencies, and thus might not work out-of-the-box on your local machine. We also probably won't be updating this repository to accomodate versions other than our own, so good luck! Feel free to submit an issue if you have any questions. 

We're using the following configuration:
* ROS Indigo
* Python 3 (we were working in Python 3.4 for our system) 
* The ROS libraries ar_track_alvar, moveit, and intera_interface

## Documentation
If you want to install TheCreator onto your own local machine for lots of Baxter and MEGABLOKS fun, view our website https://robotstudio01.cargocollective.com for thorough project descriptions, installation instructions, and package running instructions. 

To install TheCreator onto your own local machine, `cd` into your desired directory and run the following commands. The following instructions worked for the code as of 12/15/2018:
```bash
git clone https://github.com/aakarshgupta97/TheCreator
cd TheCreator/
rm -rf build/
rm -rf devel/
catkin_make
source devel/setup.bash
```

To run the kinematics test code (lab7_planning is currently a test package that allows the CV code and kinematics code to be run separately, while planning is the main integrated package that merges both the CV and kinematics):
```bash
source devel/setup.bash
catkin_make
./baxter.sh <robot_name>.local  % we used ada for our Sawyer robot, but on a generic Sawyer this should be the handle name for the robot) 
python src/lab7_planning/src/run_camera.py  % you need to be in <robot_name> to do this (do this in the same terminal as the previous step
roslaunch ar_track_alvar sawyer_ar.launch (you'll need to open a new tab and repeat the above three commands for this step and every one after this)
rosrun intera_interface enable_robot.py -e
rosrun intera_interface joint_trajectory_action_server.py
roslaunch sawyer_moveit_config sawyer_moveit.launch electric_gripper:=false
python src/lab7_planning/src/path_test.py %this should run the main body of the code
```

