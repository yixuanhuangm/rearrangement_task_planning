import numpy as np

def check_overlap(pos_a, pos_b, radius_a, radius_b):
    """Check if two disks overlap."""
    return np.linalg.norm(np.array(pos_a) - np.array(pos_b)) < (radius_a + radius_b)
