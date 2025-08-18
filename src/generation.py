import numpy as np
import matplotlib.pyplot as plt
from .disk import Disk


def generate_manual_disks(positions, goals, colors=None):
    """
    Generate disks with explicitly specified start and goal positions.

    Parameters
    ----------
    positions : list of array-like
        List of (x, y) start positions.
    goals : list of array-like
        List of (x, y) goal positions.
    colors : list or None, optional
        Colors for disks. If None, uses matplotlib's tab20 colormap.

    Returns
    -------
    list of Disk
        List of Disk instances with manual positions.
    """
    n = len(positions)

    if colors is None:
        colors = plt.cm.tab20(np.linspace(0, 1, n))

    disks = [Disk(i, colors[i], positions[i], goals[i]) for i in range(n)]
    return disks


def generate_random_disks(num_disks, x_range=(0, 1), y_range=(0, 1), seed=None, radius=0.02):
    """
    Generate disks with random start and goal positions.

    Positions are sampled uniformly within the given ranges.

    Parameters
    ----------
    num_disks : int
        Number of disks to generate.
    x_range : tuple of float, optional
        (min_x, max_x) horizontal range. Defaults to (0, 1).
    y_range : tuple of float, optional
        (min_y, max_y) vertical range. Defaults to (0, 1).
    seed : int or None, optional
        Random seed for reproducibility.
    radius : float, optional
        Radius of all disks.

    Returns
    -------
    list of Disk
        Randomly generated disks.
    """
    if seed is not None:
        np.random.seed(seed)

    colors = plt.cm.tab20(np.linspace(0, 1, num_disks))

    def sample_positions():
        xs = np.random.uniform(x_range[0], x_range[1], num_disks)
        ys = np.random.uniform(y_range[0], y_range[1], num_disks)
        return np.column_stack((xs, ys))

    starts = sample_positions()
    goals = sample_positions()

    return [Disk(i, colors[i], starts[i], goals[i], radius) for i in range(num_disks)]


def check_overlap_with_list(pos, radius, positions, radii):
    """
    Check if a position overlaps with any existing positions.

    Parameters
    ----------
    pos : array-like
        Position to check (x, y).
    radius : float
        Radius of the disk.
    positions : list of array-like
        List of existing positions.
    radii : list of float
        List of existing radii.

    Returns
    -------
    bool
        True if overlaps with any existing disk, False otherwise.
    """
    for p, r in zip(positions, radii):
        if np.linalg.norm(np.array(pos) - np.array(p)) < (radius + r):
            return True
    return False


def generate_random_disks_no_overlap(
    num_disks,
    x_range=(0, 1),
    y_range=(0, 1),
    seed=None,
    radius=0.02,
    max_attempts=1000
):
    """
    Generate disks with random start and goal positions ensuring no overlap.

    Parameters
    ----------
    num_disks : int
        Number of disks to generate.
    x_range : tuple of float, optional
        Horizontal range for positions.
    y_range : tuple of float, optional
        Vertical range for positions.
    seed : int or None, optional
        Random seed for reproducibility.
    radius : float, optional
        Radius of all disks.
    max_attempts : int, optional
        Maximum attempts to place a disk without overlap.

    Returns
    -------
    list of Disk
        Randomly generated disks without overlap.
    """
    if seed is not None:
        np.random.seed(seed)

    colors = plt.cm.tab20(np.linspace(0, 1, num_disks))
    start_positions = []
    goal_positions = []
    radii_list = []

    for i in range(num_disks):
        # Generate start position
        for _ in range(max_attempts):
            pos = np.random.uniform([x_range[0], y_range[0]], [x_range[1], y_range[1]])
            if not check_overlap_with_list(pos, radius, start_positions, radii_list):
                start_positions.append(pos)
                radii_list.append(radius)
                break
        else:
            raise ValueError(f"Cannot place disk {i} without overlap in start positions")

        # Generate goal position
        for _ in range(max_attempts):
            pos = np.random.uniform([x_range[0], y_range[0]], [x_range[1], y_range[1]])
            if not check_overlap_with_list(pos, radius, goal_positions, radii_list):
                goal_positions.append(pos)
                break
        else:
            raise ValueError(f"Cannot place disk {i} without overlap in goal positions")

    disks = [Disk(i, colors[i], start_positions[i], goal_positions[i], radius) for i in range(num_disks)]
    return disks
