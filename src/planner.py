import numpy as np
import networkx as nx
import itertools
import heapq
from .utils import *


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


def is_goal_blocked(disk, occupied_positions, disks_dict, buffer_set):
    """
    Check if a disk's goal position is blocked by other disks.
    """
    for other_id, pos in occupied_positions.items():
        if not other_id in buffer_set:
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
        if is_goal_blocked(disk, occupied_positions, disks_dict, buffer_set):
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
        if not is_goal_blocked(disk, occupied_positions, disks_dict, buffer_set):
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

    for visited_node in list(buffer_set):
        disk = disks_dict[visited_node]
        goal_pos = disk.goal_pos

        goal_blocked = any(
        check_overlap(goal_pos, occupied_positions[other_id], disk.radius, disks_dict[other_id].radius)
        for other_id in occupied_positions if other_id != node_id
        )

        if not goal_blocked:
            move_sequence.append((visited_node, 'move from buffer'))
            occupied_positions[visited_node] = goal_pos
            buffer_set.remove(visited_node)


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

def plan_unlabeled_pqs(disks):
    """
    Plan move sequence for unlabeled disks using Priority Queue Search (PQS) algorithm.

    Parameters
    ----------
    disks : list of Disk
        List of Disk instances (start and goal positions, unlabeled).

    Returns
    -------
    move_sequence : list of tuple
        Move sequence: (disk_id, action), action in ['move', 'buffer', 'move from buffer'].
    max_buffer_used : int
        Maximum number of disks simultaneously in buffer during execution (MRB).
    total_buffer_used : int
        Total number of unique disks that have been placed in buffer at least once.
    """
    n = len(disks)
    disks_dict = {disk.id: disk for disk in disks}

    goal_positions = [disk.goal_pos for disk in disks]
    occupied_positions = {disk.id: disk.start_pos for disk in disks}

    buffer_set = set()
    move_sequence = []

    goal_taken = set()
    max_buffer_used = 0
    total_buffer_used_set = set()  # Used to record the total number of unique disks that have been placed in buffer

    def compute_priority(disk_id):
        disk = disks_dict[disk_id]
        priority = 0
        for goal in goal_positions:
            if check_overlap(disk.start_pos, goal, disk.radius, 0.0):
                priority += 1
        return priority

    pq = []
    for disk_id in disks_dict.keys():
        heapq.heappush(pq, (compute_priority(disk_id), disk_id))

    while pq or buffer_set:
        moved_this_round = False

        # Try moving from buffer first
        for disk_id in list(buffer_set):
            disk = disks_dict[disk_id]
            for i, goal in enumerate(goal_positions):
                if i in goal_taken:
                    continue
                if not check_overlap_with_list(goal, disk.radius,
                                               [occupied_positions[d] for d in occupied_positions if d != disk_id],
                                               [disks_dict[d].radius for d in occupied_positions if d != disk_id]):
                    move_sequence.append((disk_id, 'move from buffer'))
                    occupied_positions[disk_id] = goal
                    goal_taken.add(i)
                    buffer_set.remove(disk_id)
                    moved_this_round = True
                    break

        if moved_this_round:
            max_buffer_used = max(max_buffer_used, len(buffer_set))
            continue

        # Take disk from priority queue
        if pq:
            _, disk_id = heapq.heappop(pq)
            disk = disks_dict[disk_id]
            for i, goal in enumerate(goal_positions):
                if i in goal_taken:
                    continue
                if not check_overlap_with_list(goal, disk.radius,
                                               [occupied_positions[d] for d in occupied_positions],
                                               [disks_dict[d].radius for d in occupied_positions]):
                    move_sequence.append((disk_id, 'move'))
                    occupied_positions[disk_id] = goal
                    goal_taken.add(i)
                    moved_this_round = True
                    break
            # If cannot move, then place into buffer
            if not moved_this_round:
                move_sequence.append((disk_id, 'buffer'))
                buffer_set.add(disk_id)
                total_buffer_used_set.add(disk_id)
                # Simple offset used as buffer position
                occupied_positions[disk_id] = [disk.start_pos[0] + 0.05, disk.start_pos[1] + 0.05]
        else:
            continue

        max_buffer_used = max(max_buffer_used, len(buffer_set))

    total_buffer_used = len(total_buffer_used_set)
    return move_sequence, max_buffer_used, total_buffer_used

def plan_unlabeled_urbm(disks):
    """
    Priority Queue-Based method for Unlabeled Running Buffer Minimization (URBM).

    Parameters
    ----------
    disks : list of Disk
        List of Disk instances.

    Returns
    -------
    move_sequence : list of tuple
        Move sequence: (disk_id, action)
    max_buffer_used : int
        Maximum simultaneously used buffer (MRB)
    total_buffer_used : int
        Total number of unique disks that ever used buffer
    """
    disks_dict = {disk.id: disk for disk in disks}
    n = len(disks)
    goal_positions = [disk.goal_pos for disk in disks]

    # State record: (priority, MRB_so_far, remaining_set, buffer_set, occupied_positions, move_sequence, total_buffer_set)
    # Priority queue sorted by priority
    start_state = (0, 0, frozenset(disks_dict.keys()), frozenset(), 
                   {disk.id: disk.start_pos for disk in disks}, [], frozenset())
    pq = []
    heapq.heappush(pq, start_state)

    visited = set()
    best_sequence = None
    best_max_buffer = None
    best_total_buffer = None

    while pq:
        priority, MRB_so_far, remaining, buffer_set, occupied_positions, move_seq, total_buffer_set = heapq.heappop(pq)

        state_key = (remaining, buffer_set)
        if state_key in visited:
            continue
        visited.add(state_key)

        # Termination condition
        if not remaining and not buffer_set:
            best_sequence = move_seq
            best_max_buffer = MRB_so_far
            best_total_buffer = len(total_buffer_set)
            break

        # Try moving from buffer to goals
        for disk_id in list(buffer_set):
            disk = disks_dict[disk_id]
            for i, goal in enumerate(goal_positions):
                if not check_overlap_with_list(goal, disk.radius,
                                               [pos for did, pos in occupied_positions.items() if did != disk_id],
                                               [disks_dict[did].radius for did in occupied_positions if did != disk_id]):
                    new_occupied = occupied_positions.copy()
                    new_occupied[disk_id] = goal
                    new_buffer = set(buffer_set)
                    new_buffer.remove(disk_id)
                    new_seq = move_seq + [(disk_id, 'move from buffer')]
                    new_total_buffer = set(total_buffer_set)
                    new_MRB = max(MRB_so_far, len(new_buffer))
                    heapq.heappush(pq, (new_MRB + len(remaining), new_MRB, remaining, frozenset(new_buffer),
                                        new_occupied, new_seq, frozenset(new_total_buffer)))
                    break  # Move once a feasible goal is found

        # Try moving remaining disks
        for disk_id in remaining:
            disk = disks_dict[disk_id]
            moved = False
            # Check if can move directly to goal
            for i, goal in enumerate(goal_positions):
                if not check_overlap_with_list(goal, disk.radius,
                                               [pos for pos in occupied_positions.values()],
                                               [disks_dict[did].radius for did in occupied_positions]):
                    new_occupied = occupied_positions.copy()
                    new_occupied[disk_id] = goal
                    new_remaining = set(remaining)
                    new_remaining.remove(disk_id)
                    new_seq = move_seq + [(disk_id, 'move')]
                    new_total_buffer = set(total_buffer_set)
                    new_MRB = max(MRB_so_far, len(buffer_set))
                    heapq.heappush(pq, (new_MRB + len(new_remaining), new_MRB, frozenset(new_remaining),
                                        buffer_set, new_occupied, new_seq, frozenset(new_total_buffer)))
                    moved = True
                    break
            # If cannot move directly, place into buffer
            if not moved:
                new_occupied = occupied_positions.copy()
                new_occupied[disk_id] = [disk.start_pos[0] + 0.05, disk.start_pos[1] + 0.05]
                new_buffer = set(buffer_set)
                new_buffer.add(disk_id)
                new_total_buffer = set(total_buffer_set)
                new_total_buffer.add(disk_id)
                new_seq = move_seq + [(disk_id, 'buffer')]
                heapq.heappush(pq, (MRB_so_far + 1, max(MRB_so_far, len(new_buffer)), remaining,
                                    frozenset(new_buffer), new_occupied, new_seq, frozenset(new_total_buffer)))

    return best_sequence, best_max_buffer, best_total_buffer

def df_dp_mrb(disks):
    """
    Depth-First Dynamic Programming (DFS + memoization) for labeled MRB.
    
    Parameters
    ----------
    disks : list of Disk
        List of Disk instances (labeled).
    
    Returns
    -------
    move_sequence : list of tuple
        Move sequence (disk_id, action)
    min_mrb : int
        Minimum running buffer required
    """
    disks_dict = {disk.id: disk for disk in disks}
    all_ids = frozenset(disks_dict.keys())
    memo = {}

    def is_goal_blocked(disk_id, occupied_positions):
        disk = disks_dict[disk_id]
        for other_id, pos in occupied_positions.items():
            if other_id != disk_id and check_overlap(disk.goal_pos, pos, disk.radius, disks_dict[other_id].radius):
                return True
        return False

    def dfs(remaining_set, occupied_positions, buffer_set):
        state_key = (remaining_set, frozenset(buffer_set))
        if state_key in memo:
            return memo[state_key]

        if not remaining_set:
            return 0, []

        min_mrb = float('inf')
        best_seq = []

        for disk_id in remaining_set:
            new_remaining = set(remaining_set)
            new_remaining.remove(disk_id)

            if is_goal_blocked(disk_id, occupied_positions):
                # Move disk to buffer if goal is blocked
                new_occupied = dict(occupied_positions)
                new_occupied[disk_id] = [occupied_positions[disk_id][0] + 0.05, occupied_positions[disk_id][1] + 0.05]
                new_buffer = set(buffer_set)
                new_buffer.add(disk_id)

                mrb, seq = dfs(frozenset(new_remaining), new_occupied, new_buffer)
                mrb = max(mrb, len(new_buffer))
                seq = [(disk_id, 'buffer')] + seq
            else:
                # Move disk directly to its goal if unblocked
                new_occupied = dict(occupied_positions)
                new_occupied[disk_id] = disks_dict[disk_id].goal_pos
                new_buffer = set(buffer_set)
                mrb, seq = dfs(frozenset(new_remaining), new_occupied, new_buffer)
                seq = [(disk_id, 'move')] + seq

            if mrb < min_mrb:
                min_mrb = mrb
                best_seq = seq

        memo[state_key] = (min_mrb, best_seq)
        return min_mrb, best_seq

    start_positions = {disk.id: disk.start_pos for disk in disks}
    min_mrb, move_sequence = dfs(all_ids, start_positions, set())
    return move_sequence, min_mrb

def get_state(S, O, G, disks_dict):
    """
    Given objects away from start poses S, return sets:
    SS: objects still at start poses
    SG: objects in S with no dependency on SS (can go to goal)
    SB: objects in S dependent on SS (must go to buffer)
    """
    SS = set(O) - set(S)
    SG = set()
    SB = set()
    for o in S:
        # if o has dependency on any object in SS, must go to buffer
        if any((o, s) in G.edges for s in SS):
            SB.add(o)
        else:
            SG.add(o)
    return SS, SG, SB


def collision_check(SS_minus_o, o, G):
    """
    Check whether object o should go to buffer.
    True = go to buffer, False = can move to goal
    """
    # If o has dependency on any object still at start pose
    for s in SS_minus_o:
        if (o, s) in G.edges:
            return True
    return False


def df_search(T, S, RB, G, O, disks_dict, buffer_usage, visited_states, move_sequence, occupied_positions):
    """
    Depth-First-Search for a fixed RB.
    """
    state_key = frozenset(S)
    if state_key in visited_states:
        return False  # Already explored

    visited_states.add(state_key)

    if set(S) == set(O):
        return True  # All objects moved to goal

    SS, SG, SB = get_state(S, O, G, disks_dict)

    for o in SS:
        SS_minus_o = SS - {o}
        to_buffer = collision_check(SS_minus_o, o, G)
        if to_buffer:
            if len(SB) + 1 > RB:
                continue  # Buffer overflow, prune
            # move to buffer
            new_buffer_usage = buffer_usage + 1
            move_sequence.append((o, 'buffer'))
        else:
            # move directly to goal
            new_buffer_usage = buffer_usage
            move_sequence.append((o, 'move'))

        occupied_positions[o] = disks_dict[o].goal_pos if not to_buffer else disks_dict[o].goal_pos  # simplify

        # Explore next state
        found = df_search(T, S | {o}, RB, G, O, disks_dict, new_buffer_usage, visited_states, move_sequence, occupied_positions)
        if found:
            return True  # Found feasible solution
        else:
            move_sequence.pop()  # backtrack

    return False

def df_dp_min_mrb(disks):
    """
    Depth-First Dynamic Programming (DFDP) to compute minimum running buffers.
    """
    disks_dict = {disk.id: disk for disk in disks}
    O = set(disks_dict.keys())
    G = nx.DiGraph()
    for disk in disks:
        G.add_node(disk.id)
    # Build dependency edges based on goal overlaps
    for d_i in disks:
        for d_j in disks:
            if d_i.id == d_j.id:
                continue
            if check_overlap(d_i.goal_pos, d_j.start_pos, d_i.radius, d_j.radius):
                G.add_edge(d_i.id, d_j.id)

    RB = 0
    while True:
        visited_states = set()
        move_sequence = []
        occupied_positions = {disk.id: disk.start_pos for disk in disks}
        buffer_usage = 0

        found = df_search(G, set(), RB, G, O, disks_dict, buffer_usage, visited_states, move_sequence, occupied_positions)
        if found:
            return RB, move_sequence  # Minimum RB found
        RB += 1