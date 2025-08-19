import networkx as nx
from .utils import check_overlap

def build_dependency_graph(disks):
    """
    Build a directed graph: edge i->j if disk i's goal overlaps disk j's start.
    """
    G = nx.DiGraph()
    for disk in disks:
        G.add_node(disk.id)

    for d_i in disks:
        for d_j in disks:
            if d_i.id != d_j.id and check_overlap(d_i.goal_pos, d_j.start_pos, d_i.radius, d_j.radius):
                G.add_edge(d_i.id, d_j.id)
    return G
