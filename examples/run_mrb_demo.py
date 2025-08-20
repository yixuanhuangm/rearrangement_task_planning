from src import *
import matplotlib.pyplot as plt

def main():
    """
    Run a demonstration of MRB disk rearrangement:
    - Generate random disks
    - Visualize static layout and dependency graph
    - Compute move sequence
    - Animate the rearrangement
    """
    # disks = generate_manual_disks(positions, goals)
    # disks = generate_random_disks_no_overlap(num_disks=20, x_range=(0, 1), y_range=(0, 1), radius=0.05)
    disks = generate_random_unlabeled_disks(num_disks=20, x_range=(0, 1), y_range=(0, 1), radius=0.05)
    visualize_disks(disks, plane_z=0)

    dependency_graph = build_dependency_graph(disks)
    visualize_dependency_graph(dependency_graph)

    sequence = plan_mrb_sequence(disks)
    print("Simple MRB Move Sequence:")
    for disk_id, action in sequence:
        print(f"Disk {disk_id}: {action}")

    sequence = plan_mrb_sequence_with_dependencies(disks)
    print("DFS MRB Move Sequence:")
    for disk_id, action in sequence:
        print(f"Disk {disk_id}: {action}")

    # sequence, mrb, mb = dp_mrb_sequence_with_logging(disks)
    # print("DPMRB Move Sequence:")
    # for disk_id, action in sequence:
    #     print(f"Disk {disk_id}: {action}")
    # print(f"MRB: {mrb}, MB: {mb}")

    sequence, mrb, mb = plan_unlabeled_pqs(disks)
    print("PQS MRB Move Sequence:")
    for disk_id, action in sequence:
        print(f"Disk {disk_id}: {action}")
    print(f"MRB: {mrb}, MB: {mb}")

    sequence, mrb, mb = plan_unlabeled_urbm(disks)
    print("URBM MRB Move Sequence:")
    for disk_id, action in sequence:
        print(f"Disk {disk_id}: {action}")
    print(f"MRB: {mrb}, MB: {mb}")

    mrb, sequence = df_dp_min_mrb(disks)
    print("DFDP2 MRB Move Sequence:")
    for disk_id, action in sequence:
        print(f"Disk {disk_id}: {action}")
    print(f"MRB: {mrb}")

    # anim = animate_mrb(disks, sequence, steps_per_action=20)
    plt.show()

if __name__ == "__main__":
    main()
