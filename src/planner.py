import numpy as np
import networkx as nx


def check_overlap(pos_a, pos_b, radius_a, radius_b):
    """
    Check if two disks overlap based on their positions and radii.

    Parameters
    ----------
    pos_a : array-like
        Position of the first disk (x, y).
    pos_b : array-like
        Position of the second disk (x, y).
    radius_a : float
        Radius of the first disk.
    radius_b : float
        Radius of the second disk.

    Returns
    -------
    bool
        True if disks overlap, False otherwise.
    """
    return np.linalg.norm(np.array(pos_a) - np.array(pos_b)) < (radius_a + radius_b)


def build_dependency_graph(disks):
    """
    Build a directed graph representing dependencies between disks.
    Edge (i, j) indicates disk i's goal overlaps with disk j's start position.

    Parameters
    ----------
    disks : list of Disk
        List of Disk instances.

    Returns
    -------
    nx.DiGraph
        Directed graph of dependencies.
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
    Check if a disk's goal position is blocked by other disks.

    Parameters
    ----------
    disk : Disk
        Disk to check.
    current_positions : dict
        Current positions {disk_id: position}.
    disks_dict : dict
        Dictionary of disks {disk_id: Disk}.

    Returns
    -------
    bool
        True if the goal position is blocked, False otherwise.
    """
    for other_id, pos in current_positions.items():
        if other_id != disk.id and check_overlap(disk.goal_pos, pos, disk.radius, disks_dict[other_id].radius):
            return True
    return False


def move_from_origin(remaining_set, buffer_set, current_positions, disks_dict, move_sequence, buffer):
    """
    Attempt to move disks from their original positions.

    If a disk's goal is blocked, move it to a buffer position instead.

    Parameters
    ----------
    remaining_set : set
        Set of remaining disk IDs.
    buffer_set : set
        Set of disk IDs currently in buffer.
    current_positions : dict
        Current positions {disk_id: position}.
    disks_dict : dict
        Dictionary of disks {disk_id: Disk}.
    move_sequence : list
        List to store movement actions.
    buffer : dict
        Buffer positions {disk_id: position}.

    Returns
    -------
    bool
        True if at least one disk was moved, False otherwise.
    """
    moved = False
    for disk_id in list(remaining_set):
        if disk_id in buffer_set:
            continue

        disk = disks_dict[disk_id]
        if is_goal_blocked(disk, current_positions, disks_dict):
            move_sequence.append((disk_id, 'buffer'))
            current_positions[disk_id] = buffer[disk_id]
            buffer_set.add(disk_id)
            moved = True
        else:
            move_sequence.append((disk_id, 'move'))
            current_positions[disk_id] = disk.goal_pos
            remaining_set.remove(disk_id)
            moved = True

        return moved


def move_from_buffer(buffer_set, current_positions, disks_dict, move_sequence, remaining_set):
    """
    Attempt to move disks from buffer to their goal positions if unblocked.

    Parameters
    ----------
    buffer_set : set
        Set of disk IDs currently in buffer.
    current_positions : dict
        Current positions {disk_id: position}.
    disks_dict : dict
        Dictionary of disks {disk_id: Disk}.
    move_sequence : list
        List to store movement actions.
    remaining_set : set
        Set of remaining disk IDs.

    Returns
    -------
    bool
        True if at least one disk was moved, False otherwise.
    """
    moved = False
    for disk_id in list(buffer_set):
        disk = disks_dict[disk_id]
        if not is_goal_blocked(disk, current_positions, disks_dict):
            move_sequence.append((disk_id, 'move from buffer'))
            current_positions[disk_id] = disk.goal_pos
            buffer_set.remove(disk_id)
            remaining_set.remove(disk_id)
            moved = True

    return moved


def plan_mrb_sequence(disks):
    """
    Plan the move sequence for multiple disks using MRB (Move-Remove-Buffer) strategy.

    Parameters
    ----------
    disks : list of Disk
        List of Disk instances.

    Returns
    -------
    list of tuple
        List of movement actions in the form (disk_id, action), where action is 'move' or 'buffer'.
    """
    disks_dict = {disk.id: disk for disk in disks}
    remaining = set(disks_dict.keys())
    buffer_set = set()
    move_sequence = []
    current_positions = {disk.id: disk.start_pos for disk in disks}

    # Generate buffer positions
    buffer = generate_buffer_positions(disks, current_positions, gap=2.0)

    while remaining or buffer_set:
        print(f"Buffer: {buffer_set}, Buffer Size: {len(buffer_set)}")
        if move_from_buffer(buffer_set, current_positions, disks_dict, move_sequence, remaining):
            continue
        move_from_origin(remaining, buffer_set, current_positions, disks_dict, move_sequence, buffer)

    return move_sequence


def generate_buffer_positions(disks, start_pos, gap=1.5):
    """
    Generate buffer positions for disks to temporarily move into.

    Parameters
    ----------
    disks : list of Disk
        List of disks.
    start_pos : dict
        Dictionary of current positions {disk_id: position}.
    gap : float
        Minimum distance between buffer positions.

    Returns
    -------
    dict
        Buffer positions {disk_id: buffer_pos}.
    """
    buffer_pos = {}
    x_offset = 0
    y_offset = 0

    for disk in disks:
        # Simple linear arrangement along y=0
        buffer_pos[disk.id] = [start_pos[disk.id][0] + x_offset, start_pos[disk.id][1] + y_offset]
        x_offset += gap

    return buffer_pos
