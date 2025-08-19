import itertools
from .utils import check_overlap
from .planner import generate_buffer_positions

def dp_mrb(disks):
    """Compute minimum running buffers (MRB) using DP and return optimal sequence."""
    n = len(disks)
    disks_dict = {i: disk for i, disk in enumerate(disks)}
    ids = list(disks_dict.keys())

    from .dependency_graph import build_dependency_graph
    G = build_dependency_graph(disks)

    T = {0: {'b': set(), 'MRB': 0, 'parent': None}}
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
                RB = max(T[parent_state]['MRB'], len(T[parent_state]['b']) + 1) if oi in b_set else T[parent_state]['MRB']
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
        move_sequence.append(disks_dict[moved_disk].id)
        state = parent

    move_sequence = move_sequence[::-1]
    return min_mrb, move_sequence

def dp_mrb_sequence(disks):
    """Convert DP sequence to buffer/move actions."""
    min_mrb, sequence = dp_mrb(disks)
    disks_dict = {disk.id: disk for disk in disks}
    occupied_positions = {disk.id: disk.start_pos for disk in disks}
    buffer_set = set()
    move_sequence = []
    buffer_zone = generate_buffer_positions(disks, occupied_positions, gap=2.0)

    for disk_id in sequence:
        disk = disks_dict[disk_id]
        goal_blocked = any(check_overlap(disk.goal_pos, occupied_positions[other_id], disk.radius, disks_dict[other_id].radius)
                           for other_id in occupied_positions if other_id != disk_id)
        if goal_blocked:
            move_sequence.append((disk_id, 'buffer'))
            occupied_positions[disk_id] = buffer_zone[disk_id]
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
