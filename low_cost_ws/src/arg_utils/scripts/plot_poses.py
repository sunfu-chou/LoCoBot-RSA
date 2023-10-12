# Examples in pytransform3d https://dfki-ric.github.io/pytransform3d/_auto_examples/index.html

import numpy as np
import matplotlib.pyplot as plt

from pytransform3d.transformations import plot_transform
from pytransform3d.plot_utils import make_3d_axis
import pytransform3d.camera as pc
import pytransform3d.transformations as pt
from pytransform3d import rotations as pr
from pytransform3d.plot_utils import remove_frame

ax = make_3d_axis(ax_s=1, unit="m", n_ticks=6)
plot_transform(ax=ax)
plt.tight_layout()
plt.show()


cam2world = pt.transform_from_pq([0, 0, 0, np.sqrt(0.5), -np.sqrt(0.5), 0, 0])
# default parameters of a camera in Blender
sensor_size = np.array([0.036, 0.024])
intrinsic_matrix = np.array([
    [0.05, 0, sensor_size[0] / 2.0],
    [0, 0.05, sensor_size[1] / 2.0],
    [0, 0, 1]
])
virtual_image_distance = 1

ax = pt.plot_transform(A2B=cam2world, s=0.2)
pc.plot_camera(
    ax, cam2world=cam2world, M=intrinsic_matrix, sensor_size=sensor_size,
    virtual_image_distance=virtual_image_distance)
plt.show()



alpha, beta, gamma = 0.5 * np.pi, 0.5 * np.pi, 0.5 * np.pi
p = np.array([1, 1, 1])

plt.figure(figsize=(5, 5))

ax = pr.plot_basis(R=np.eye(3), p=-1.5 * p, ax_s=2)
pr.plot_axis_angle(ax, [1, 0, 0, alpha], -1.5 * p)

pr.plot_basis(
    ax, pr.active_matrix_from_extrinsic_euler_xyz([alpha, 0, 0]), -0.5 * p)
pr.plot_axis_angle(ax, [0, 1, 0, beta], p=-0.5 * p)

pr.plot_basis(
    ax, pr.active_matrix_from_extrinsic_euler_xyz([alpha, beta, 0]), 0.5 * p)
pr.plot_axis_angle(ax, [0, 0, 1, gamma], 0.5 * p)

pr.plot_basis(
    ax,
    pr.active_matrix_from_extrinsic_euler_xyz([alpha, beta, gamma]), 1.5 * p,
    lw=5)

remove_frame(ax)

plt.show()
