import numpy as np
import matplotlib.pyplot as plt


class Block():
    def __init__(self, coords, layer_num):
        self.coords = coords
        self.layer_num = layer_num


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
        # get_strat_by_layer()

    def compute_strategy(self):
        for layer in self.creation.layers:
        	self.strategy.append([b.coords for b in layer.blocks])
        return np.array(self.strategy)


def main():
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

   model = Creation(test)
   strat = Strategy(model)
   strategy = strat.compute_strategy()

   # np.save('strategy', strategy)
   np.save(srcPath + '/blocks/' + file_name, layers)

   plt.imshow(test)
   plt.show()


   
if __name__ == '__main__':
    main()
    
