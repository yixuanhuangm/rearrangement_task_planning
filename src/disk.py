import numpy as np

class Disk:
    """
    Represents a circular disk with start and goal positions.

    Parameters
    ----------
    disk_id : int
        Unique identifier for the disk.
    color : array-like
        Color specification (e.g., RGBA) for visualization.
    start_position : array-like
        Initial (x, y) position of the disk.
    goal_position : array-like
        Target (x, y) position for the disk.
    radius : float, optional
        Radius of the disk. Default is 0.02.

    Attributes
    ----------
    id : int
        Disk identifier.
    color : ndarray
        Color data converted to numpy array.
    start_pos : ndarray
        Numpy array of the start position.
    goal_pos : ndarray
        Numpy array of the goal position.
    radius : float
        Disk radius.
    """
    def __init__(self, disk_id, color, start_position, goal_position, radius=0.02):
        self.id = disk_id
        self.color = color
        self.start_pos = np.array(start_position)
        self.goal_pos = np.array(goal_position)
        self.radius = radius
