#!/usr/bin/env python
"""
Main function for TheCreator
Authors: Ahad Rauf, Aakarsh Gupta, Amay Saxena, Sairanjith Thalanki
"""

import sys
import rospy
import numpy as np
from planning.srv import *

def modelGeneratorClient():
	rospy.wait_for_service('model_generator', ModelGenerator)
	try:
		model_generator = rospy.ServiceProxy('model_generator', ModelGenerator)
		model_generator_response = model_generator()
		errorCode, blocks = model_generator_response.errorCode, model_generator_response.blocks
		return errorCode, blocks
	except rospy.ServiceException as e:
		print("Service call failed:", str(e))

def buildStructure(blocks):
	rospy.wait_for_service('build_structure', BuildStructure)
	try:
		build_structure = rospy.ServiceProxy('build_structure', BuildStructure)
		build_structure_response = build_structure(blocks)
		errorCode = build_structure_response.errorCode
		return errorCode
	except rospy.ServiceException as e:
		print("Service call failed:", str(e))

if __name__ == "__main__":
	# Use this format if you want to get command line parameters
	# if len(sys.argv) == 3:
	# 	x, y = *sys.argv[1:]
	errorCode1, blocks = retrieveBlocks()
	if errorCode1 != 0:
		raise ValueError("Retrieving blocks gave the error code", errorCode1)

	errorCode2 = buildStructure(blocks)
	if errorCode2 != 0:
		raise ValueError("Retrieving blocks gave the error code", errorCode2)

	print('Finished. Praise me now.')
