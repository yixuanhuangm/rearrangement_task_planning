import numpy as np

def check_overlap(pos_a, pos_b, radius_a, radius_b):
    """Check if two disks overlap."""
    return np.linalg.norm(np.array(pos_a) - np.array(pos_b)) < (radius_a + radius_b)

def check_overlap_with_list(pos, radius, positions, radii):
    """
    Check if a position overlaps with any existing positions.

    Parameters
    ----------
    pos : array-like
        Position to check (x, y).
    radius : float
        Radius of the disk to check.
    positions : list of array-like
        List of existing positions.
    radii : list of float
        List of radii corresponding to the positions.

    Returns
    -------
    bool
        True if `pos` overlaps with any position in `positions`, False otherwise.
    """
    for p, r in zip(positions, radii):
        if check_overlap(pos, p, radius, r):
            return True
    return False
