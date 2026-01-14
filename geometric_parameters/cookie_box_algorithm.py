# cookie_box - Geometric Analysis Algorithm
# Primitives: {'component': ['box']}

import numpy as np
import open3d as o3d
from sklearn.decomposition import PCA

def extract_parameters(points):
    # Compute bounds and initial center
    min_bounds = np.min(points, axis=0)
    max_bounds = np.max(points, axis=0)
    center = (min_bounds + max_bounds) / 2

    # Perform PCA to determine orientation
    centered = points - center
    pca = PCA(n_components=3)
    pca.fit(centered)
    rotation_matrix = pca.components_.T

    # Ensure right-handed coordinate system
    if np.linalg.det(rotation_matrix) < 0:
        rotation_matrix[:, 2] *= -1

    # Check alignment with world axes
    threshold = 0.95
    x_aligned = max(abs(rotation_matrix[0, 0]), abs(rotation_matrix[1, 0]), abs(rotation_matrix[2, 0])) > threshold
    y_aligned = max(abs(rotation_matrix[0, 1]), abs(rotation_matrix[1, 1]), abs(rotation_matrix[2, 1])) > threshold
    z_aligned = max(abs(rotation_matrix[0, 2]), abs(rotation_matrix[1, 2]), abs(rotation_matrix[2, 2])) > threshold
    is_axis_aligned = x_aligned and y_aligned and z_aligned

    # Compute dimensions
    if not is_axis_aligned:
        # Tilted - compute dimensions in local frame
        local_points = centered @ rotation_matrix
        min_local = np.min(local_points, axis=0)
        max_local = np.max(local_points, axis=0)
        dimensions = max_local - min_local
    else:
        # Axis-aligned - use global frame dimensions
        dimensions = max_bounds - min_bounds

    # Add margin to ensure enclosure
    dimensions *= 1.02

    if not is_axis_aligned:
        return {
            'parameters': {
                'component_0': {
                    'type': 'box',
                    'center': center.tolist(),
                    'dimensions': dimensions.tolist(),
                    'rotation_matrix': rotation_matrix.tolist()
                }
            }
        }
    else:
        return {
            'parameters': {
                'component_0': {
                    'type': 'box',
                    'center': center.tolist(),
                    'dimensions': dimensions.tolist()
                }
            }
        }

def create_visualization_geometry(params):
    geometries = []
    
    for comp_name, comp_params in params['parameters'].items():
        if comp_params['type'] == 'box':
            center = np.array(comp_params['center'])
            dims = np.array(comp_params['dimensions'])
            
            mesh = o3d.geometry.TriangleMesh.create_box(
                width=dims[0], height=dims[1], depth=dims[2]
            )
            
            if 'rotation_matrix' in comp_params:
                # Oriented box
                R = np.array(comp_params['rotation_matrix'])
                mesh.translate(-dims/2)
                mesh.rotate(R, center=[0,0,0])
                mesh.translate(center)
            else:
                # Axis-aligned box
                mesh.translate(center - dims/2)
            
            mesh.paint_uniform_color([0.7, 0.3, 0.3])
            mesh.compute_vertex_normals()
            geometries.append(mesh)
    
    return geometries