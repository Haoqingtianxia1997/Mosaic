# bowl - Geometric Analysis Algorithm
# Primitives: {'component': ['hemisphere']}

import numpy as np
import open3d as o3d
from sklearn.decomposition import PCA

def extract_parameters(points):
    # Compute bounds and initial center estimate
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

    # Compute radius and determine opening direction
    local_points = centered @ rotation_matrix
    radius = np.max(np.sqrt(local_points[:, 0]**2 + local_points[:, 1]**2 + local_points[:, 2]**2))
    
    # Analyze point cloud shape to determine opening direction
    top_points = points[centered[:, 2] > 0]
    bottom_points = points[centered[:, 2] < 0]
    
    if len(top_points) > 10 and len(bottom_points) > 10:
        top_center_xy = np.mean(top_points[:, :2], axis=0)
        bottom_center_xy = np.mean(bottom_points[:, :2], axis=0)
        
        top_radius = np.max(np.linalg.norm(top_points[:, :2] - top_center_xy, axis=1))
        bottom_radius = np.max(np.linalg.norm(bottom_points[:, :2] - bottom_center_xy, axis=1))
        
        opening_at_top = top_radius > bottom_radius * 1.1
    else:
        opening_at_top = len(top_points) > len(bottom_points)

    # For hemisphere, center should be at the opening plane (rim level)
    if opening_at_top:
        axis_dir = np.array([0, 0, 1]) if is_axis_aligned else rotation_matrix[:, 2]
        center[2] = max_bounds[2] if is_axis_aligned else center[2] + radius * 0.3
    else:
        axis_dir = np.array([0, 0, -1]) if is_axis_aligned else -rotation_matrix[:, 2]
        center[2] = min_bounds[2] if is_axis_aligned else center[2] - radius * 0.3

    radius *= 1.02

    return {
        'parameters': {
            'component_0': {
                'type': 'hemisphere',
                'center': center.tolist(),
                'radius': float(radius),
                'axis': axis_dir.tolist()
            }
        }
    }

def create_visualization_geometry(params):
    geometries = []
    for comp_name, comp_params in params['parameters'].items():
        if comp_params['type'] == 'hemisphere':
            center = np.array(comp_params['center'])
            radius = comp_params['radius']
            
            mesh = o3d.geometry.TriangleMesh.create_sphere(radius=radius, resolution=20)
            vertices = np.asarray(mesh.vertices)
            triangles = np.asarray(mesh.triangles)
            
            axis = np.array(comp_params.get('axis', [0, 0, 1]))
            axis = axis / np.linalg.norm(axis)
            
            projections = vertices @ axis
            threshold = 0.1 * radius
            keep_mask = projections <= threshold
            
            old_to_new = np.full(len(vertices), -1)
            old_to_new[keep_mask] = np.arange(np.sum(keep_mask))
            new_vertices = vertices[keep_mask]
            
            new_triangles = []
            for tri in triangles:
                if np.all(keep_mask[tri]):
                    new_triangles.append([old_to_new[tri[0]], old_to_new[tri[1]], old_to_new[tri[2]]])
            
            if len(new_triangles) > 0:
                mesh = o3d.geometry.TriangleMesh()
                mesh.vertices = o3d.utility.Vector3dVector(new_vertices)
                mesh.triangles = o3d.utility.Vector3iVector(np.array(new_triangles))
                
                default_axis = np.array([0, 0, 1])
                if np.abs(np.dot(default_axis, axis) - 1.0) > 1e-6:
                    v = np.cross(default_axis, axis)
                    s = np.linalg.norm(v)
                    c = np.dot(default_axis, axis)
                    if s > 1e-6:
                        vx = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
                        R = np.eye(3) + vx + vx @ vx * ((1 - c) / (s**2))
                        mesh.rotate(R, center=[0,0,0])
                
                mesh.translate(center)
                mesh.paint_uniform_color([0.9, 0.9, 0.85])
                mesh.compute_vertex_normals()
                geometries.append(mesh)
    
    return geometries