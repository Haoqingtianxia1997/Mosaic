# teacup - Geometric Analysis Algorithm
# Primitives: {'component': ['cylinder']}

import numpy as np
import open3d as o3d
from sklearn.decomposition import PCA

def extract_parameters(points):
    # Step 1: Compute bounds and initial center
    min_bounds = np.min(points, axis=0)
    max_bounds = np.max(points, axis=0)
    center = (min_bounds + max_bounds) / 2

    # Step 2: Compute PCA for orientation
    centered = points - center
    pca = PCA(n_components=3)
    pca.fit(centered)
    rotation_matrix = pca.components_.T

    # Ensure right-handed coordinate system
    if np.linalg.det(rotation_matrix) < 0:
        rotation_matrix[:, 2] *= -1

    # Step 3: Check alignment with world axes
    threshold = 0.95
    x_aligned = max(abs(rotation_matrix[0, 0]), abs(rotation_matrix[1, 0]), abs(rotation_matrix[2, 0])) > threshold
    y_aligned = max(abs(rotation_matrix[0, 1]), abs(rotation_matrix[1, 1]), abs(rotation_matrix[2, 1])) > threshold
    z_aligned = max(abs(rotation_matrix[0, 2]), abs(rotation_matrix[1, 2]), abs(rotation_matrix[2, 2])) > threshold
    is_axis_aligned = x_aligned and y_aligned and z_aligned

    # Step 4: Compute parameters for cylinder (cup body)
    local_points = centered @ rotation_matrix
    height = np.max(local_points[:, 0]) - np.min(local_points[:, 0])
    radius = np.max(np.sqrt(local_points[:, 1]**2 + local_points[:, 2]**2))
    radius *= 1.02
    height *= 1.02

    if not is_axis_aligned:
        # Tilted cylinder - store axis direction
        cylinder_axis = rotation_matrix[:, 0]
        return {
            'parameters': {
                'component_0': {
                    'type': 'cylinder',
                    'center': center.tolist(),
                    'radius': float(radius),
                    'height': float(height),
                    'axis': cylinder_axis.tolist()
                }
            }
        }
    else:
        # Upright cylinder
        return {
            'parameters': {
                'component_0': {
                    'type': 'cylinder',
                    'center': center.tolist(),
                    'radius': float(radius),
                    'height': float(height)
                }
            }
        }

def create_visualization_geometry(params):
    geometries = []
    for comp_name, comp_params in params['parameters'].items():
        if comp_params['type'] == 'cylinder':
            center = np.array(comp_params['center'])
            radius = comp_params['radius']
            height = comp_params['height']
            mesh = o3d.geometry.TriangleMesh.create_cylinder(radius=radius, height=height)
            if 'axis' in comp_params:
                axis = np.array(comp_params['axis'])
                # Compute rotation from [0,0,1] to axis
                z_axis = np.array([0, 0, 1])
                v = np.cross(z_axis, axis)
                s = np.linalg.norm(v)
                c = np.dot(z_axis, axis)
                vx = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
                R = np.eye(3) + vx + vx @ vx * ((1 - c) / (s ** 2 + 1e-10))
                mesh.rotate(R, center=[0,0,0])
            mesh.translate(center)
            mesh.paint_uniform_color([0.7, 0.3, 0.3])
            mesh.compute_vertex_normals()
            geometries.append(mesh)
    return geometries