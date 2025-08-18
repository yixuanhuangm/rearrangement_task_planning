import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from .utils import draw_disk_3d

def animate_mrb(disks, move_sequence, steps_per_action=20):
    """
    Animate the MRB disk rearrangement process in 3D.

    Parameters
    ----------
    disks : list of Disk
        List of disks to animate.
    move_sequence : list of tuple (disk_id, str)
        Sequence of disk actions ('move' or 'buffer').
    steps_per_action : int, optional
        Number of interpolation frames per action. Default is 20.

    Returns
    -------
    matplotlib.animation.FuncAnimation
        Animation object for interactive display.

    中文说明
    --------
    在3D中动态展示MRB圆盘重排过程，含缓冲和移动动画。
    """
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.set_xlim(0, 1.2); ax.set_ylim(0, 1); ax.set_zlim(0, 0.05)
    ax.set_xlabel('X'); ax.set_ylabel('Y'); ax.set_zlabel('Z')
    ax.set_title('MRB Disk Rearrangement Animation')

    disks_dict = {disk.id: disk for disk in disks}
    positions = {disk.id: disk.start_pos.copy() for disk in disks}
    buffer_set = set()
    frames = []

    for disk_id, action in move_sequence:
        disk = disks_dict[disk_id]
        start = positions[disk_id].copy()
        end = disk.goal_pos if action == 'move' else np.array([1.05 + 0.05 * disk_id, 0.8])
        if action == 'buffer':
            buffer_set.add(disk_id)
        for t in np.linspace(0, 1, steps_per_action):
            interpolated_positions = {**positions}
            interpolated_positions[disk_id] = (1 - t) * start + t * end
            frames.append((interpolated_positions.copy(), buffer_set.copy()))
        positions[disk_id] = end
        if action == 'move' and disk_id in buffer_set:
            buffer_set.remove(disk_id)

    def update_frame(frame_idx):
        ax.cla()
        ax.set_xlim(0, 1.2); ax.set_ylim(0, 1); ax.set_zlim(0, 0.05)
        ax.set_xlabel('X'); ax.set_ylabel('Y'); ax.set_zlabel('Z')
        ax.set_title('MRB Disk Rearrangement Animation')
        pos_map, current_buffer = frames[frame_idx]
        for d_id, pos in pos_map.items():
            disk = disks_dict[d_id]
            z_offset = 0.02 if d_id in current_buffer else 0
            draw_disk_3d(ax, pos, disk.radius, disk.color, alpha=0.6, z=z_offset)
        return ax.collections

    anim = FuncAnimation(fig, update_frame, frames=len(frames), interval=100, blit=False)
    plt.show(block=False)
    return anim
