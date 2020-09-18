import numpy as np


class RandomDestination:
    def __init__(self, x_max, y_max):
        self.density = np.load("density_map.npy")
        self.shape = self.density.shape
        self.density = self.density.flatten()
        self.density /= np.sum(self.density)

        self.xscale = x_max/self.shape[1]
        self.yscale = y_max/self.shape[0]

    def random(self):
        r = np.random.choice(self.density.shape[0], p=self.density)
        y, x = np.unravel_index(r, self.shape)
        return int(x*self.xscale), int((self.shape[0]-y)*self.yscale)
