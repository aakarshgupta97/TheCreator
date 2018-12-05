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

def retrieveBlocks():
	# rospy.wait_for_service('model_generator', ModelGenerator, timeout=10)
	rospy.wait_for_service('model_generator')
	try:
		model_generator = rospy.ServiceProxy('model_generator', ModelGenerator)
		model_generator_response = model_generator()
		errorCode = model_generator_response.errorCode
		blocks = model_generator_response.blocks
		width = model_generator_response.width
		height = model_generator_response.height
		return errorCode, blocks, width, height
	except rospy.ServiceException as e:
		print("Service call failed:", str(e))

def buildStructure(blocks):
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
	errorCode1, blocks, width, height = retrieveBlocks()
	if errorCode1 != 0:
		raise ValueError("Retrieving blocks gave the error code", errorCode1)

	errorCode2 = buildStructure(blocks, width, height)
	if errorCode2 != 0:
		raise ValueError("Retrieving blocks gave the error code", errorCode2)

	print('Finished. Praise me now.')
