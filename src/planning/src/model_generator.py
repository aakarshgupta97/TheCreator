#!/usr/bin/env python
"""
Model generator for TheCreator
Authors: Ahad Rauf, Aakarsh Gupta, Amay Saxena, Sairanjith Thalanki
"""

import numpy as np
import matplotlib.pyplot as plt

import rospy

import os, sys
planningPath = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
sys.path.append(planningPath)
srvPath = os.path.join(planningPath, 'srv')
sys.path.append(srvPath)
from planning.srv import ModelGenerator, ModelGeneratorResponse

def generate_model(depth_map):
    height = np.amax(depth_map)
    layer_blocks = []
    for i in range(1, height + 1):
        layer = np.zeros_like(depth_map)
        layer[depth_map >= i] = 1
        layer_blocks.append(get_block_structure(layer))


def get_block_structure(layer):
    blocks = []
    col = 2
    for i in range(len(layer)):
        for j in range(len(layer[0])):
            if layer[i, j] == 1:
                blocks.append((i, j))
                layer[i, j] = col
                layer[i + 1, j] = col
                layer[i, j + 1] = col
                layer[i + 1, j + 1] = col
                col += 1
    plt.imshow(layer)
    plt.show()
    return blocks

def modelGeneratorMain():
   test = np.zeros((10, 12)).astype(np.int32)
   test[3][6] = 1
   test[3][7] = 1
   test[4][4] = 1
   test[4][5] = 1
   test[4][6] = 1
   test[4][7] = 1
   test[5][4] = 1
   test[5][5] = 1
   test[5][6] = 1
   test[5][7] = 1
   test[6][6] = 1
   test[6][7] = 1
   test[7][4] = 1
   test[7][5] = 1
   test[8][4] = 1
   test[8][5] = 1

   errorCode = 0
   blocks = generate_model(test)
   return ModelGeneratorResponse(errorCode, blocks, 10, 12)
   
def initialize_service():
	rospy.init_node('model_generator_node')
	service = rospy.Service('model_generator', ModelGenerator, modelGeneratorMain)
	print('Ready to create model')
	rospy.spin()

if __name__ == '__main__':
    initialize_service()