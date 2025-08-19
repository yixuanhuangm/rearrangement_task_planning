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
    disks = generate_random_disks_no_overlap(num_disks=20, x_range=(0, 1), y_range=(0, 1), radius=0.05)
    # visualize_disks(disks, plane_z=0)

    dependency_graph = build_dependency_graph(disks)
    visualize_dependency_graph(dependency_graph)

    sequence = plan_mrb_sequence(disks)
    print("MRB Move Sequence:")
    for disk_id, action in sequence:
        print(f"Disk {disk_id}: {action}")

    sequence = plan_mrb_sequence_with_dependencies(disks)
    print("MRB Move Sequence:")
    for disk_id, action in sequence:
        print(f"Disk {disk_id}: {action}")

    sequence, mrb, mb = dp_mrb_sequence_with_logging(disks)
    print("MRB Move Sequence:")
    for disk_id, action in sequence:
        print(f"Disk {disk_id}: {action}")
    print(f"MRB: {mrb}, MB: {mb}")

    # anim = animate_mrb(disks, sequence, steps_per_action=20)
    plt.show()

if __name__ == "__main__":
    main()
