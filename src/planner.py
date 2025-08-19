import numpy as np
import networkx as nx
import itertools


def check_overlap(pos_a, pos_b, radius_a, radius_b):
    """
    Check if two disks overlap based on their positions and radii.
    """
    return np.linalg.norm(np.array(pos_a) - np.array(pos_b)) < (radius_a + radius_b)


def build_dependency_graph(disks):
    """
    Build a directed graph representing dependencies between disks.
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


def is_goal_blocked(disk, occupied_positions, disks_dict):
    """
    Check if a disk's goal position is blocked by other disks.
    """
    for other_id, pos in occupied_positions.items():
        if other_id != disk.id and check_overlap(disk.goal_pos, pos, disk.radius, disks_dict[other_id].radius):
            return True
    return False


def move_from_origin(remaining_set, buffer_set, occupied_positions, disks_dict, move_sequence, buffer_zone):
    """
    Attempt to move disks from their original positions.
    """
    moved = False
    for disk_id in list(remaining_set):
        if disk_id in buffer_set:
            continue

        disk = disks_dict[disk_id]
        if is_goal_blocked(disk, occupied_positions, disks_dict):
            move_sequence.append((disk_id, 'buffer'))
            occupied_positions[disk_id] = buffer_zone[disk_id]
            buffer_set.add(disk_id)
            moved = True
        else:
            move_sequence.append((disk_id, 'move'))
            occupied_positions[disk_id] = disk.goal_pos
            remaining_set.remove(disk_id)
            moved = True

        return moved


def move_from_buffer(buffer_set, occupied_positions, disks_dict, move_sequence, remaining_set):
    """
    Attempt to move disks from buffer to their goal positions if unblocked.
    """
    moved = False
    for disk_id in list(buffer_set):
        disk = disks_dict[disk_id]
        if not is_goal_blocked(disk, occupied_positions, disks_dict):
            move_sequence.append((disk_id, 'move from buffer'))
            occupied_positions[disk_id] = disk.goal_pos
            buffer_set.remove(disk_id)
            remaining_set.remove(disk_id)
            moved = True

    return moved


def plan_mrb_sequence(disks):
    """
    Plan the move sequence for multiple disks using MRB strategy.
    """
    disks_dict = {disk.id: disk for disk in disks}
    remaining = set(disks_dict.keys())
    buffer_set = set()
    move_sequence = []
    occupied_positions = {disk.id: disk.start_pos for disk in disks}

    buffer_zone = generate_buffer_positions(disks, occupied_positions, gap=2.0)

    while remaining or buffer_set:
        print(f"Buffer: {buffer_set}, Buffer Size: {len(buffer_set)}")
        if move_from_buffer(buffer_set, occupied_positions, disks_dict, move_sequence, remaining):
            continue
        move_from_origin(remaining, buffer_set, occupied_positions, disks_dict, move_sequence, buffer_zone)

    return move_sequence


def generate_buffer_positions(disks, start_pos, gap=1.5):
    """
    Generate buffer positions for disks to temporarily move into.
    """
    buffer_pos = {}
    x_offset = 0
    y_offset = 0

    for disk in disks:
        buffer_pos[disk.id] = [start_pos[disk.id][0] + x_offset, start_pos[disk.id][1] + y_offset]
        x_offset += gap

    return buffer_pos


def move_with_dependencies(node_id, disks_dict, occupied_positions, buffer_set, buffer_zone, move_sequence, remaining_set, visited, G):
    """
    Recursively move a disk and all its dependencies.
    """
    if not node_id in remaining_set:
        return
    if node_id in visited:
        return

    visited.add(node_id)

    for dep in G.successors(node_id):
        if dep not in visited:
            move_with_dependencies(dep, disks_dict, occupied_positions, buffer_set, buffer_zone, move_sequence, remaining_set, visited, G)

    disk = disks_dict[node_id]
    goal_pos = disk.goal_pos

    goal_blocked = any(
        check_overlap(goal_pos, occupied_positions[other_id], disk.radius, disks_dict[other_id].radius)
        for other_id in occupied_positions if other_id != node_id
    )

    if goal_blocked:
        move_sequence.append((node_id, 'buffer'))
        occupied_positions[node_id] = buffer_zone[node_id]
        buffer_set.add(node_id)
    else:
        move_sequence.append((node_id, 'move'))
        occupied_positions[node_id] = goal_pos
        remaining_set.remove(node_id)


def plan_mrb_sequence_with_dependencies(disks):
    disks_dict = {disk.id: disk for disk in disks}
    remaining_set = set(disks_dict.keys())
    buffer_set = set()
    move_sequence = []
    occupied_positions = {disk.id: disk.start_pos for disk in disks}

    buffer_zone = generate_buffer_positions(disks, occupied_positions, gap=2.0)
    G = build_dependency_graph(disks)
    visited = set()

    for node_id in disks_dict.keys():
        if node_id not in visited:
            move_with_dependencies(node_id, disks_dict, occupied_positions, buffer_set, buffer_zone, move_sequence, remaining_set, visited, G)

    for disk_id in list(buffer_set):
        disk = disks_dict[disk_id]
        move_sequence.append((disk_id, 'move from buffer'))
        occupied_positions[disk_id] = disk.goal_pos
        buffer_set.remove(disk_id)

    return move_sequence


def dp_mrb(disks):
    """
    DP algorithm strictly following the pseudocode from the paper.
    """
    n = len(disks)
    disks_dict = {i: disk for i, disk in enumerate(disks)}
    ids = list(disks_dict.keys())
    G = build_dependency_graph(disks)

    T = dict()
    T[0] = {'b': set(), 'MRB': 0, 'parent': None}

    for k in range(1, n+1):
        for subset in itertools.combinations(ids, k):
            state = sum(1<<i for i in subset)
            T[state] = {}
            b_set = set()
            subset_set = set(subset)
            for o in subset:
                for o2 in ids:
                    if o2 not in subset_set and G.has_edge(o, o2):
                        b_set.add(o)
                        break
            T[state]['b'] = b_set
            T[state]['MRB'] = float('inf')
            T[state]['parent'] = None

            for oi in subset:
                parent_state = state ^ (1<<oi)
                if oi in T[state]['b']:
                    RB = max(T[parent_state]['MRB'], len(T[parent_state]['b']) + 1)
                else:
                    RB = T[parent_state]['MRB']
                if RB < T[state]['MRB']:
                    T[state]['MRB'] = RB
                    T[state]['parent'] = parent_state

    full_state = (1<<n) - 1
    min_mrb = T[full_state]['MRB']

    move_sequence = []
    state = full_state
    while state != 0:
        parent = T[state]['parent']
        moved_disk = (state ^ parent).bit_length() - 1
        move_sequence.append(moved_disk)
        state = parent
    move_sequence = move_sequence[::-1]
    move_sequence = [disks_dict[i].id for i in move_sequence]

    return min_mrb, move_sequence


def dp_mrb_sequence(disks):
    """
    Generate MRB move sequence using DP and map it to buffer/move actions.
    """
    min_mrb, sequence = dp_mrb(disks)
    disks_dict = {disk.id: disk for disk in disks}
    occupied_positions = {disk.id: disk.start_pos for disk in disks}
    buffer_set = set()
    move_sequence = []
    buffer = generate_buffer_positions(disks, occupied_positions, gap=2.0)

    for disk_id in sequence:
        disk = disks_dict[disk_id]
        goal_blocked = any(
            check_overlap(disk.goal_pos, occupied_positions[other_id], disk.radius, disks_dict[other_id].radius)
            for other_id in occupied_positions if other_id != disk_id
        )

        if goal_blocked:
            move_sequence.append((disk_id, 'buffer'))
            occupied_positions[disk_id] = buffer[disk_id]
            buffer_set.add(disk_id)
        else:
            move_sequence.append((disk_id, 'move'))
            occupied_positions[disk_id] = disk.goal_pos

    for disk_id in list(buffer_set):
        disk = disks_dict[disk_id]
        move_sequence.append((disk_id, 'move from buffer'))
        occupied_positions[disk_id] = disk.goal_pos
        buffer_set.remove(disk_id)

    return move_sequence, min_mrb


def dp_mrb_sequence_with_logging(disks, gap=2.0, verbose=True):
    """
    Generate MRB move sequence using DP with logging of buffer usage.
    """
    min_mrb, sequence = dp_mrb(disks)
    disks_dict = {disk.id: disk for disk in disks}
    occupied_positions = {disk.id: disk.start_pos for disk in disks}
    buffer_set = set()
    move_sequence = []
    max_buffer_used = 0
    buffer = generate_buffer_positions(disks, occupied_positions, gap=gap)

    for disk_id in sequence:
        disk = disks_dict[disk_id]
        goal_blocked = any(
            check_overlap(disk.goal_pos, occupied_positions[other_id], disk.radius, disks_dict[other_id].radius)
            for other_id in occupied_positions if other_id != disk_id
        )

        if goal_blocked:
            move_sequence.append((disk_id, 'buffer'))
            occupied_positions[disk_id] = buffer[disk_id]
            buffer_set.add(disk_id)
        else:
            move_sequence.append((disk_id, 'move'))
            occupied_positions[disk_id] = disk.goal_pos

        max_buffer_used = max(max_buffer_used, len(buffer_set))
        if verbose:
            print(f"Step: Move disk {disk_id}, action: {'buffer' if goal_blocked else 'move'}, "
                  f"Current buffer: {buffer_set}, Max buffer: {max_buffer_used}")

    for disk_id in list(buffer_set):
        disk = disks_dict[disk_id]
        move_sequence.append((disk_id, 'move from buffer'))
        occupied_positions[disk_id] = disk.goal_pos
        buffer_set.remove(disk_id)
        max_buffer_used = max(max_buffer_used, len(buffer_set))
        if verbose:
            print(f"Step: Move disk {disk_id} from buffer, "
                  f"Current buffer: {buffer_set}, Max buffer: {max_buffer_used}")

    return move_sequence, min_mrb, max_buffer_used
