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

import numpy as np
import matplotlib.pyplot as plt


class Block():
    def __init__(self, coords, layer_num):
        self.coords = coords
        self.layer_num = layer_num

    def get_constraints(layer):
        u, v = self.coords
        left = layer[u - 1, v]


class Layer():
    def __init__(self, layer, layer_num):
        self.arr = layer
        self.layer_num = layer_num
        self.blocks = []
        self.get_block_structure()

    def get_block_structure(self):
        layer_copy = np.zeros_like(self.arr)
        layer_copy[:, :] = self.arr[:, :]
        for i in range(len(layer_copy)):
            for j in range(len(layer_copy[0])):
                if layer_copy[i, j] == 1:
                    self.blocks.append(Block((i, j), self.layer_num))
                    layer_copy[i, j] = 0
                    layer_copy[i + 1, j] = 0
                    layer_copy[i, j + 1] = 0
                    layer_copy[i + 1, j + 1] = 0


class Creation():
    def __init__(self, depth_map):
        self.layers = self.generate_layers(depth_map)
        self.num_layers = len(self.layers)

    def generate_layers(self, depth_map):
        height = np.amax(depth_map)
        layers = []
        for i in range(1, height + 1):
            layer = np.zeros_like(depth_map)
            layer[depth_map >= i] = 1
            layers.append((Layer(layer, i)))
        return layers

class Strategy():
    def __init__(self, creation):
        self.creation = creation
        self.strategy = []
        self.constraints = {}
        get_strat_by_layer()

    def get_constraints(self, block):
        block_layer = self.creation.layers[block.layer_num - 1].arr
        u, v = block.coords
        cons = ""
        neigh = [[(u, v - 1) , (u + 1, v - 1)], [(u - 1, v), (u - 1, v + 1)], [(u, v + 2)]]
        is_valid = lambda i, j: (i in range(10)) and (j in range(12))
   
        for side, coords in zip('lur', neigh):
            for c in coords:
                i, j = c
                if is_valid(i, j) and block_layer[i, j] == 1:
                    cons += side  
        self.constraints[(u, v)] = cons

    def get_strat_by_layer(self, layer):
        for block in layer.blocks:
            self.get_constraints(block)


def modelGeneratorMain(input):
	"""
	Input should be empty
	"""
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

	# model = Creation(test, (0, 0, 0))
	model = Creation(test)
	blocks = model.layers

	print(blocks)
	# blocks = [layer.blocks() for layer in blocks]

	width = len(test)
	height = len(test[0])
	num_layers = np.amax(depth_map)
	layers = []
    for i in range(1, num_layers + 1):
        layer = np.zeros_like(depth_map)
        layer[depth_map >= i] = 1


    np.save()

	blocks = list(np.ndarray.flatten(np.array(blocks)))
	return ModelGeneratorResponse(blocks, num_layers, width, height)
    
   
def initialize_service():
	rospy.init_node('model_generator_node')
	service = rospy.Service('model_generator', ModelGenerator, modelGeneratorMain)
	print('Ready to create model')
	rospy.spin()

if __name__ == '__main__':
    initialize_service()
