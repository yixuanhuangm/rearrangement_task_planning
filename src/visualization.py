import matplotlib.pyplot as plt
import networkx as nx
from .utils import draw_disk_3d

def visualize_disks(disks, plane_z=0):
    """
    Visualize disks in 3D with start and goal positions connected by dashed lines.

    Parameters
    ----------
    disks : list of Disk
        The disks to visualize.
    plane_z : float, optional
        Z-height for drawing disks; default is 0.

    Notes
    -----
    Start positions are drawn opaque, goal positions semi-transparent, with
    dashed lines indicating intended movement.
    """
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    for disk in disks:
        draw_disk_3d(ax, disk.start_pos, disk.radius, disk.color, alpha=0.8, z=plane_z)
        ax.text(*disk.start_pos, plane_z, str(disk.id), color='k')
        draw_disk_3d(ax, disk.goal_pos, disk.radius * 0.8, disk.color, alpha=0.3, z=plane_z)
        ax.text(*disk.goal_pos, plane_z, str(disk.id), color='k')
        ax.plot([disk.start_pos[0], disk.goal_pos[0]],
                [disk.start_pos[1], disk.goal_pos[1]],
                [plane_z, plane_z],
                color=disk.color, linestyle='--')
    ax.set_xlim(0, 1.2); ax.set_ylim(0, 1); ax.set_zlim(0, 0.05)
    ax.set_xlabel('X'); ax.set_ylabel('Y'); ax.set_zlabel('Z')
    ax.set_title('Disk Rearrangement (Static)')
    plt.show(block=False)

def visualize_dependency_graph(graph):
    """
    Draw a directed dependency graph using NetworkX.

    Parameters
    ----------
    graph : networkx.DiGraph
        Graph representing dependencies among disk moves.

    Notes
    -----
    Nodes represent disks; directed edges indicate blocking dependencies.
    """
    pos = nx.spring_layout(graph)
    plt.figure()
    nx.draw(graph, pos, with_labels=True, node_color='skyblue', node_size=1000, arrowsize=20)
    plt.title("Dependency Graph")
    plt.show(block=False)
