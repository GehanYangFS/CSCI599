from multiprocessing.spawn import import_main_path


import numpy as np
def normalize(v):
    norm = np.linalg.norm(v)
    if norm == 0:
        norm = np.finfo(v.dtype).eps
    return v/norm