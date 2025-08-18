import numpy as np
import networkx as nx

def check_overlap(pos_a, pos_b, radius_a, radius_b):
    """
    Determine if two disks overlap based on their positions and radii.

    Parameters
    ----------
    pos_a : array-like
        (x, y) coordinates of disk A center.
    pos_b : array-like
        (x, y) coordinates of disk B center.
    radius_a : float
        Radius of disk A.
    radius_b : float
        Radius of disk B.

    Returns
    -------
    bool
        True if disks overlap, False otherwise.
    """
    return np.linalg.norm(np.array(pos_a) - np.array(pos_b)) < (radius_a + radius_b)

def build_dependency_graph(disks):
    """
    Construct a directed graph representing move dependencies:
    an edge from A to B indicates A must move before B.

    Parameters
    ----------
    disks : list of Disk
        List of disk objects.

    Returns
    -------
    networkx.DiGraph
        Directed graph with disks as nodes and blocking dependencies as edges.
    """
    G = nx.DiGraph()
    for disk in disks:
        G.add_node(disk.id)
    for d_i in disks:
        for d_j in disks:
            if d_i.id == d_j.id:
                continue
            if check_overlap(d_i.goal_pos, d_j.start_pos, d_i.radius, d_j.radius):
                G.add_edge(d_i.id, d_j.id)
    return G

def is_goal_blocked(disk, current_positions, disks_dict):
    """
    Check if a disk's goal position is blocked by another disk.

    Parameters
    ----------
    disk : Disk
        The disk to check.
    current_positions : dict
        Mapping from disk ID to its current position.
    disks_dict : dict
        Mapping from disk ID to Disk object.

    Returns
    -------
    bool
        True if the goal position is blocked; False otherwise.
    """
    for other_id, pos in current_positions.items():
        if other_id != disk.id and check_overlap(disk.goal_pos, pos, disk.radius, disks_dict[other_id].radius):
            return True
    return False

def schedule_buffered_disks(buffer_set, current_positions, disks_dict, move_sequence, remaining_set):
    """
    Attempt to move disks in the buffer that are no longer blocked.

    Parameters
    ----------
    buffer_set : set of int
        Disk IDs currently in buffer.
    current_positions : dict
        Disk ID -> current position mapping.
    disks_dict : dict
        Disk ID -> Disk object mapping.
    move_sequence : list of tuple (disk_id, str)
        Recorded sequence of actions ('move' or 'buffer').
    remaining_set : set of int
        Disk IDs yet to be processed.

    Returns
    -------
    bool
        True if at least one disk moved this round; False otherwise.
    """
    moved = False
    for disk_id in buffer_set:
        disk = disks_dict[disk_id]
        if not is_goal_blocked(disk, current_positions, disks_dict):
            move_sequence.append((disk_id, 'move'))
            current_positions[disk_id] = disk.goal_pos
            buffer_set.remove(disk_id)
            remaining_set.remove(disk_id)
            moved = True
        
    return moved

def simple_schedule_remaining_disks(remaining_set, buffer_set, current_positions, disks_dict, move_sequence):
    """
    Process disks not yet moved or buffered: move or buffer them based on blockage.

    Parameters
    ----------
    remaining_set : set of int
        Disk IDs awaiting processing.
    buffer_set : set of int
        Disk IDs in buffer.
    current_positions : dict
        Disk ID -> current position mapping.
    disks_dict : dict
        Disk ID -> Disk object mapping.
    move_sequence : list of tuple (disk_id, str)
        Recorded sequence of actions.

    Returns
    -------
    bool
        True if a disk was processed; False otherwise.
    """
    for disk_id in list(remaining_set):
        disk = disks_dict[disk_id]
        if is_goal_blocked(disk, current_positions, disks_dict):
            move_sequence.append((disk_id, 'buffer'))
            buffer_set.add(disk_id)
        else:
            move_sequence.append((disk_id, 'move'))
            current_positions[disk_id] = disk.goal_pos
            remaining_set.remove(disk_id)
        return True
    return False

def plan_mrb_sequence(disks):
    """
    Plan the move sequence based on the Minimum Rearrangement Buffer (MRB) logic.

    Parameters
    ----------
    disks : list of Disk
        List of disks to rearrange.

    Returns
    -------
    list of tuple (disk_id, str)
        Ordered sequence of actions: ('move' or 'buffer') for each disk.

    Notes
    -----
    Implements a greedy scheduling strategy: prioritize buffered disks if unblocked;
    otherwise handle a remaining disk. Detects deadlocks if no progress.
    """
    disks_dict = {disk.id: disk for disk in disks}
    remaining = set(disks_dict.keys())
    buffer_set = set()
    move_sequence = []
    current_positions = {disk.id: disk.start_pos for disk in disks}

    while remaining or buffer_set:
        if schedule_buffered_disks(buffer_set, current_positions, disks_dict, move_sequence, remaining):
            continue
        if simple_schedule_remaining_disks(remaining, buffer_set, current_positions, disks_dict, move_sequence):
            continue
        print("Warning: deadlock detected!")
        break

    return move_sequence
