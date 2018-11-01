# TheCreator
The Creator is a general-purpose LEGO 

##Dependencies
TheCreator is designed for our current versions of ROS, Python, and other dependencies, and thus might not work out-of-the-box on your local machine. We also probably won't be updating this repository to accomodate versions other than our own, so good luck!

We're using the following configuration:
* ROS Indigo
* We've created a submodule of the [https://github.com/RightHandRobotics/reflex-ros-pkg](RightHand ReFlex gripper hand package), which you can update by calling the command `git submodule update`. Our version was downloaded on 11/1/2018 at roughly 3:45PM, PST.

## Installation
TheCreator is currently a private repository. However, if you do manage to gain access, you can install it onto your own local machine for lots of Baxter and LEGO fun:

To install TheCreator onto your own local machine, `cd` into your desired directory and run the following commands:
```bash
git clone https://github.com/aakarshgupta97/TheCreator
git submodule update
```
