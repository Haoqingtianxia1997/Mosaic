# tomato - Geometric Analysis Algorithm
# Primitives: {'component': ['ellipsoid']}

import numpy as np
import open3d as o3d
from sklearn.decomposition import PCA

def extract_parameters(points):
    # Step 1: Compute initial center
    min_bounds = np.min(points, axis=0)
    max_bounds = np.max(points, axis=0)
    center = (min_bounds + max_bounds) / 2
    
    # Step 2: Compute PCA to check orientation
    centered = points - center
    pca = PCA(n_components=3)
    pca.fit(centered)
    rotation_matrix = pca.components_.T
    if np.linalg.det(rotation_matrix) < 0:
        rotation_matrix[:, 2] *= -1
    
    # Step 3: Check alignment with world axes
    threshold = 0.95
    x_aligned = max(abs(rotation_matrix[0, 0]), abs(rotation_matrix[1, 0]), abs(rotation_matrix[2, 0])) > threshold
    y_aligned = max(abs(rotation_matrix[0, 1]), abs(rotation_matrix[1, 1]), abs(rotation_matrix[2, 1])) > threshold
    z_aligned = max(abs(rotation_matrix[0, 2]), abs(rotation_matrix[1, 2]), abs(rotation_matrix[2, 2])) > threshold
    is_axis_aligned = x_aligned and y_aligned and z_aligned
    
    # Step 4: Compute radii in local frame
    local_points = centered @ rotation_matrix
    radii = np.array([np.max(np.abs(local_points[:, i])) for i in range(3)])
    radii *= 1.02  # Add margin
    
    # Step 5: Return parameters based on alignment
    if not is_axis_aligned:
        return {
            'parameters': {
                'component_0': {
                    'type': 'ellipsoid',
                    'center': center.tolist(),
                    'radii': radii.tolist(),
                    'rotation_matrix': rotation_matrix.tolist()
                }
            }
        }
    else:
        return {
            'parameters': {
                'component_0': {
                    'type': 'ellipsoid',
                    'center': center.tolist(),
                    'radii': radii.tolist()
                }
            }
        }

def create_visualization_geometry(params):
    geometries = []
    for comp_name, comp_params in params['parameters'].items():
        if comp_params['type'] == 'ellipsoid':
            center = np.array(comp_params['center'])
            radii = np.array(comp_params['radii'])
            mesh = o3d.geometry.TriangleMesh.create_sphere(radius=1.0)
            mesh.scale(radii[0], center=[0,0,0])
            vertices = np.asarray(mesh.vertices)
            vertices = vertices / radii[0] * radii.reshape(1, 3)
            mesh.vertices = o3d.utility.Vector3dVector(vertices)
            if 'rotation_matrix' in comp_params:
                R = np.array(comp_params['rotation_matrix'])
                mesh.rotate(R, center=[0,0,0])
            mesh.translate(center)
            mesh.paint_uniform_color([0.7, 0.3, 0.3])
            mesh.compute_vertex_normals()
            geometries.append(mesh)
    return geometries