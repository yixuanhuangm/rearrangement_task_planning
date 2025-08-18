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
