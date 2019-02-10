import numpy as np

def make_map():
    ret =  np.zeros((3, 3), dtype = "uint8")
    ret[1, 2] = 1
    return ret
