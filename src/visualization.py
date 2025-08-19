import matplotlib.pyplot as plt
import networkx as nx
import numpy as np
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

def draw_disk_3d(ax, center, radius, color, alpha=0.6, z=0):
    """
    Draw a filled circle (disk) on a 3D axis at a specified z-height.

    Parameters
    ----------
    ax : matplotlib.axes._subplots.Axes3DSubplot
        The 3D axes to draw on.
    center : array-like
        (x, y) center of the disk.
    radius : float
        Radius of the disk.
    color : color spec
        Disk color.
    alpha : float, optional
        Opacity level; default is 0.6.
    z : float, optional
        Height at which to draw the disk; default is 0.

    Returns
    -------
    Poly3DCollection
        The created polygon collection for further customization.
    """
    theta = np.linspace(0, 2*np.pi, 50)
    x = center[0] + radius * np.cos(theta)
    y = center[1] + radius * np.sin(theta)
    z_coords = np.full_like(x, z)
    verts = [list(zip(x, y, z_coords))]
    collection = Poly3DCollection(verts, color=color, alpha=alpha)
    ax.add_collection3d(collection)
    return collection

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
    nx.draw(graph, pos, with_labels=True, node_color='skyblue', node_size=500, arrowsize=20)
    plt.title("Dependency Graph")
    plt.show(block=False)
