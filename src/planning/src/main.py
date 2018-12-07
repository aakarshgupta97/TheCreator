#!/usr/bin/env python
"""
Main function for TheCreator
Authors: Ahad Rauf, Aakarsh Gupta, Amay Saxena, Sairanjith Thalanki
"""

import sys
import rospy
import numpy as np

import os, sys
planningPath = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
print(planningPath)
sys.path.append(planningPath)
srvPath = os.path.join(planningPath, 'srv')
print(srvPath)
sys.path.append(srvPath)
from planning.srv import *

def switchMaster(new_master):
	rospy.wait_for_service('switch_master')
	try:
		build_structure = rospy.ServiceProxy('switch_master', SwitchMaster)
		build_structure_response = build_structure(new_master)
		errorCode = build_structure_response.errorCode
		return errorCode
	except rospy.ServiceException as e:
		print("Service call failed:", str(e))

def retrieveBlocks():
	# rospy.wait_for_service('model_generator', ModelGenerator, timeout=10)
	rospy.wait_for_service('model_generator')
	try:
		model_generator = rospy.ServiceProxy('model_generator', ModelGenerator)
		print(model_generator)
		model_generator_response = model_generator()
		errorCode = model_generator_response.errorCode
		blocks = model_generator_response.blocks
		width = model_generator_response.width
		height = model_generator_response.height
		return errorCode, blocks, width, height
	except rospy.ServiceException as e:
		print("Service call failed:", str(e))

def buildStructure(blocks, width, height):
	# rospy.wait_for_service('build_structure', BuildStructure)
	rospy.wait_for_service('build_structure')
	try:
		build_structure = rospy.ServiceProxy('build_structure', BuildStructure)
		build_structure_response = build_structure(blocks, width, height)
		errorCode = build_structure_response.errorCode
		return errorCode
	except rospy.ServiceException as e:
		print("Service call failed:", str(e))

if __name__ == "__main__":
	# Use this format if you want to get command line parameters
	# if len(sys.argv) == 3:
	# 	x, y = *sys.argv[1:]
	errorCode0 = switchMaster('black')
	if errorCode0 != 0:
		raise ValueError("Switching master gave the error code", errorCode0)

	import os
	import paramiko  # For SSH'ing into turtlebot
	print("$ROS_MASTER_URI = ", os.environ["ROS_MASTER_URI"])
	# ssh = paramiko.SSHClient()
	# ssh.connect('black.local', username='turtlebot', password='EE106A18')
	# ssh_stdin, ssh_stdout, ssh_stderr = ssh.exec_command("print('Hello')")
	os.environ["ROS_MASTER_URI"] = 'http://yellow.local:11311'
	HOST="turtlebot@yellow.local"
	# Ports are handled in ~/.ssh/config since we use OpenSSH
	COMMAND="pwd; cat ~/.bashrc; echo $ROS_MASTER_URI; rostopic list;"

	import subprocess
	ssh = subprocess.Popen(["ssh", "%s" % HOST, COMMAND],
	                       shell=False,
	                       stdout=subprocess.PIPE,
	                       stderr=subprocess.PIPE)
	result = ssh.stdout.readlines()
	if result == []:
	    error = ssh.stderr.readlines()
	    print (sys.stderr, "ERROR: %s" % error)
	else:
	    for r in result:
	    	print(r.strip())

	# errorCode1, blocks, width, height = retrieveBlocks()
	# if errorCode1 != 0:
	# 	raise ValueError("Retrieving blocks gave the error code", errorCode1)

	# print(blocks, width, height)
	# import sys
	# sys.stdout.flush()

	# errorCode2 = buildStructure(blocks, width, height)
	# if errorCode2 != 0:
	# 	raise ValueError("Retrieving blocks gave the error code", errorCode2)

	print('Finished. Praise me now.')
