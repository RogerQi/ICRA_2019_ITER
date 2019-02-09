import arena_config
import numpy as np

class arena:
    def __init__(self):
        self.__map__ = np.zeros((arena_config.cell_y_cnt, arena_config.cell_x_cnt))
