import numpy as np
import open3d as o3d
from scipy.spatial.transform import Rotation as R

# left extrinsic parameters(acquired from startup_launch.py)
# translation = np.array([0.14287264529286604, -0.5160394374662502, 0.5285747070512808])
# roll = 0.4719635237773772
# pitch = 0.8017128013264066
# yaw = 1.0319360545389382

# right extrinsic parameters(acquired from startup_launch.py)
translation = np.array([0.8595406021203789, 0.5037965577635406, 0.5703258113488032])
qx = 0.37301026915950647
qy = 0.06207915901055871
qz = -0.8924183137659721
qw = 0.24616878431920047

# world coordinate frame(large arrows)
world_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=1, origin=[0, 0, 0])

# rot = R.from_euler('xyz', [roll, pitch, yaw]).as_matrix()
rot = R.from_quat([qx, qy, qz, qw]).as_matrix()
# 4x4 transformation matrix
T = np.eye(4)
T[:3, :3] = rot
T[:3, 3] = translation

# camera coordinate frame (small arrows)
transformed_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1, origin=[0, 0, 0])
transformed_frame.transform(T)

# Visualize
o3d.visualization.draw_geometries([world_frame, transformed_frame])