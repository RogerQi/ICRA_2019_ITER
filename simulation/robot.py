import sys
sys.path.append("../.")
import numpy as np
from utils.make_map import make_map

class robot:
    def __init__(self):
        self._map = make_map()
        self.tar_x = 0
        self.tar_y = 0

    def set_target_loc(self, x, y):
        self.tar_x = x
        self.tar_y = y

    def get_current_loc(self):
        self.x = 0 # TODO: get data from server
        self.y = 0
