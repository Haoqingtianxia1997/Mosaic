"""
Visualization Module - Visualize point clouds, geometries and trajectories using Open3D
"""
import numpy as np
import open3d as o3d
import json
import os


class ResultVisualizer:
    """Result Visualizer - Display point clouds, geometric parameterization results and trajectories"""
    
    def __init__(self):
        """Initialize visualizer"""
        self.geometries = []
    
    def create_coordinate_frame(self, size=0.1, origin=[0, 0, 0]):
        """Create coordinate frame"""
        return o3d.geometry.TriangleMesh.create_coordinate_frame(
            size=size, origin=origin
        )
    
    def create_parametric_geometry(self, object_name, params, params_file_path=None):
        """
        Create parametric geometry based on geometric parameters
        
        Load LLM-generated visualization function from corresponding algorithm.py file
        
        Args:
            object_name: Object name
            params: Geometric parameters dictionary
            params_file_path: Parameters file path (used to locate algorithm.py)
            
        Returns:
            List of Open3D geometry objects
        """
        geometries = []
        
        # Load LLM-generated visualization function from algorithm.py
        if params_file_path:
            algorithm_file = params_file_path.replace('_params.json', '_algorithm.py')
        else:
            # If params_file_path not provided, try to infer
            params_dir = "geometric_parameters"
            algorithm_file = os.path.join(params_dir, f"{object_name}_algorithm.py")
        
        if os.path.exists(algorithm_file):
            try:
                namespace = {'np': np, 'o3d': o3d, 'numpy': np}
                with open(algorithm_file, 'r', encoding='utf-8') as f:
                    algorithm_code = f.read()
                exec(algorithm_code, namespace)
                
                if 'create_visualization_geometry' in namespace:
                    vis_func = namespace['create_visualization_geometry']
                    # Ensure passing parameters in correct format (containing 'parameters' key)
                    if 'parameters' not in params:
                        params_dict = {'parameters': params}
                    else:
                        params_dict = params
                    
                    geoms = vis_func(params_dict)
                    if geoms:
                        print(f"  ✓ Created {len(geoms)} geometries using LLM-generated visualization function")
                        return geoms
                else:
                    print(f"  ✗ Warning: No create_visualization_geometry function in {algorithm_file}")
            except Exception as e:
                print(f"  ✗ Failed to load visualization function from algorithm file: {e}")
                import traceback
                traceback.print_exc()
        else:
            print(f"  ✗ Warning: Cannot find algorithm file: {algorithm_file}")
        
        # If failed, return empty list
        print(f"  ⚠ Unable to create visualization geometry")
        return geometries
    
    def create_trajectory_visualization(self, trajectory, color=None):
        """
        Create trajectory visualization (unified color scheme)
        
        Args:
            trajectory: Trajectory waypoint list
            color: Reserved parameter for backward compatibility, but will be ignored
            
        Returns:
            List of Open3D geometry objects
            
        Unified color scheme:
            - 🟢 Green large sphere [0, 1, 0]: Start point (1st waypoint)
            - 🔴 Red large sphere [1, 0, 0]: End point (last waypoint)
            - 🔵 Blue small spheres [0, 0, 1]: Intermediate waypoints (all other waypoints)
            - 🔵 Blue line [0, 0, 1]: Trajectory line connecting all waypoints
            
        Note: All trajectories use the same color scheme, regardless of object type
        """
        geometries = []
        
        if trajectory is None or len(trajectory) == 0:
            return geometries
        
        # Extract positions
        positions = []
        for waypoint in trajectory:
            if isinstance(waypoint, dict) and 'position' in waypoint:
                pos = waypoint['position']
                if isinstance(pos, (list, np.ndarray)) and len(pos) >= 3:
                    positions.append(pos[:3])
        
        if len(positions) < 2:
            return geometries
        
        positions = np.array(positions)
        
        # Unified color scheme
        TRAJECTORY_LINE_COLOR = [0, 0, 1]     # Blue line
        START_POINT_COLOR = [0, 1, 0]          # Green start point
        END_POINT_COLOR = [1, 0, 0]            # Red end point
        WAYPOINT_COLOR = [0, 0, 1]             # Blue intermediate waypoints
        
        # Create trajectory line (blue)
        points = o3d.utility.Vector3dVector(positions)
        lines = [[i, i+1] for i in range(len(positions)-1)]
        line_set = o3d.geometry.LineSet(
            points=points,
            lines=o3d.utility.Vector2iVector(lines)
        )
        line_set.paint_uniform_color(TRAJECTORY_LINE_COLOR)
        geometries.append(line_set)
        
        # Create sphere markers at each waypoint
        for i, pos in enumerate(positions):
            # Start and end points use large spheres, intermediate waypoints use small spheres
            if i == 0 or i == len(positions) - 1:
                radius = 0.010  # Start and end: larger spheres, ensure visibility
            else:
                radius = 0.005  # Intermediate waypoints: small spheres
            
            sphere = o3d.geometry.TriangleMesh.create_sphere(radius=radius, resolution=15)
            sphere.translate(pos)
            
            # Unified color scheme
            if i == 0:
                sphere.paint_uniform_color(START_POINT_COLOR)  # Start point: green
            elif i == len(positions) - 1:
                sphere.paint_uniform_color(END_POINT_COLOR)    # End point: red
                # Make end point more visible - add a slightly larger outer circle
                outer_sphere = o3d.geometry.TriangleMesh.create_sphere(radius=radius*1.3, resolution=15)
                outer_sphere.translate(pos)
                outer_sphere.paint_uniform_color([1, 0.5, 0.5])  # Light red outer circle
                outer_sphere.compute_vertex_normals()
                geometries.append(outer_sphere)
            else:
                sphere.paint_uniform_color(WAYPOINT_COLOR)     # Intermediate: blue
            
            sphere.compute_vertex_normals()
            geometries.append(sphere)
        
        return geometries
    
    def create_legend(self, position=[0, 0, 0.3], scale=0.02):
        """
        Create legend explaining trajectory color meanings
        
        Args:
            position: Legend position [x, y, z]
            scale: Legend size scale
            
        Returns:
            List of Open3D geometry objects
        """
        geometries = []
        offset_z = 0
        spacing = scale * 2
        
        # Start point example (green)
        start_sphere = o3d.geometry.TriangleMesh.create_sphere(radius=scale, resolution=10)
        start_sphere.translate([position[0], position[1], position[2] + offset_z])
        start_sphere.paint_uniform_color([0, 1, 0])
        start_sphere.compute_vertex_normals()
        geometries.append(start_sphere)
        offset_z -= spacing
        
        # Intermediate waypoint example (blue)
        middle_sphere = o3d.geometry.TriangleMesh.create_sphere(radius=scale*0.7, resolution=10)
        middle_sphere.translate([position[0], position[1], position[2] + offset_z])
        middle_sphere.paint_uniform_color([0, 0, 1])
        middle_sphere.compute_vertex_normals()
        geometries.append(middle_sphere)
        offset_z -= spacing
        
        # End point example (red)
        end_sphere = o3d.geometry.TriangleMesh.create_sphere(radius=scale, resolution=10)
        end_sphere.translate([position[0], position[1], position[2] + offset_z])
        end_sphere.paint_uniform_color([1, 0, 0])
        end_sphere.compute_vertex_normals()
        geometries.append(end_sphere)
        
        return geometries
    
    def visualize_single_result(self, object_name, point_cloud_path, params, trajectory=None, params_file_path=None):
        """
        Visualize complete results for a single object
        
        Args:
            object_name: Object name
            point_cloud_path: Point cloud file path or list of paths
            params: Geometric parameters dictionary
            trajectory: Trajectory waypoint list
            params_file_path: Parameters file path (used to load visualization function)
        """
        print(f"\nVisualizing {object_name}...")
        
        geometries = []
        
        # 1. Load point cloud(s)
        point_cloud_paths = [point_cloud_path] if isinstance(point_cloud_path, str) else point_cloud_path
        total_points = 0
        for path in point_cloud_paths:
            if os.path.exists(path):
                pcd = o3d.io.read_point_cloud(path)
                total_points += len(pcd.points)
                geometries.append(pcd)
            else:
                print(f"  - Warning: Point cloud file does not exist: {path}")
        
        if total_points > 0:
            print(f"  - Loaded point cloud(s): {total_points} points from {len(point_cloud_paths)} file(s)")
        
        # 2. Create parametric geometry (pass parameter file path)
        param_geoms = self.create_parametric_geometry(object_name, params, params_file_path)
        print(f"  - Created parametric geometries: {len(param_geoms)}")
        geometries.extend(param_geoms)
        
        # 3. Create trajectory visualization
        if trajectory:
            traj_geoms = self.create_trajectory_visualization(trajectory)
            print(f"  - Created trajectory visualization: {len(trajectory)} waypoints")
            print(f"    Color legend: 🟢Green=start, 🔴Red=end, others=intermediate waypoints")
            geometries.extend(traj_geoms)
        
        # 4. Add coordinate frame
        coord_frame = self.create_coordinate_frame(size=0.05)
        geometries.append(coord_frame)
        
        # 5. Visualize
        print(f"  - Starting visualization window...")
        o3d.visualization.draw_geometries(
            geometries,
            window_name=f"{object_name} - Point Cloud, Geometry & Trajectory",
            width=1200,
            height=900,
            left=50,
            top=50
        )
    
    def visualize_all_results(self, results_dict, point_cloud_dir="point_cloud", params_dir="geometric_parameters"):
        """
        Visualize results of all objects (displayed side by side)
        
        Args:
            results_dict: Dictionary containing all results
            point_cloud_dir: Point cloud file directory
            params_dir: Geometric parameters file directory
            
        All trajectories use unified color scheme:
            - 🟢 Green large sphere = Start point of all trajectories
            - 🔴 Red large sphere = End point of all trajectories
            - 🔵 Blue small spheres = Intermediate waypoints of all trajectories
            - 🔵 Blue line = Motion path of all trajectories
        """
        print("\n=== Starting Visualization of All Results ===")
        print("\nTrajectory Color Legend (Unified Color Scheme):")
        print("  🟢 Green large sphere = Start point (starting position of all trajectories)")
        print("  🔴 Red large sphere = End point (ending position of all trajectories)")
        print("  🔵 Blue small spheres = Intermediate waypoints (process positions of trajectories)")
        print("  🔵 Blue line = Motion path\n")
        
        all_geometries = []
        offset_x = 0
        spacing = 0.4  # Object spacing
        
        for idx, (object_name, result) in enumerate(results_dict.items()):
            print(f"\nProcessing {object_name}...")
            
            # 1. Load point cloud(s)
            point_cloud_paths = result['parameterization']['point_cloud_paths']
            total_points = 0
            for path in point_cloud_paths:
                if os.path.exists(path):
                    pcd = o3d.io.read_point_cloud(path)
                    pcd.translate([offset_x, 0, 0])
                    all_geometries.append(pcd)
                    total_points += len(pcd.points)
            
            if total_points > 0:
                print(f"  ✓ Point cloud(s): {total_points} points from {len(point_cloud_paths)} file(s)")
            
            # 2. Create parametric geometry
            params = result['parameterization']['parameters']
            # Now use JSON format parameter file
            params_file_path = os.path.join(params_dir, f"{object_name}_params.json")
            param_geoms = self.create_parametric_geometry(object_name, params, params_file_path)
            for geom in param_geoms:
                geom.translate([offset_x, 0, 0])
                all_geometries.append(geom)
            print(f"  ✓ Geometries: {len(param_geoms)}")
            
            # 3. Create trajectory (unified color scheme, no need to pass color parameter)
            trajectory = result['trajectory'].get('trajectory')
            if trajectory:
                traj_geoms = self.create_trajectory_visualization(trajectory)
                for geom in traj_geoms:
                    geom.translate([offset_x, 0, 0])
                    all_geometries.append(geom)
                print(f"  ✓ Trajectory: {len(trajectory)} waypoints (green start→blue intermediate→red end)")
            
            # 4. Add coordinate frame
            coord = self.create_coordinate_frame(size=0.03, origin=[offset_x, 0, 0])
            all_geometries.append(coord)
            
            # Update offset
            offset_x += spacing
        
        # Add global coordinate frame
        global_coord = self.create_coordinate_frame(size=0.08, origin=[0, 0, 0])
        all_geometries.append(global_coord)
        
        # Visualize all objects
        print("\nStarting visualization window...")
        o3d.visualization.draw_geometries(
            all_geometries,
            window_name="All Objects - Point Clouds, Geometries & Trajectories",
            width=1600,
            height=1000,
            left=50,
            top=50
        )
    
    def save_visualization_screenshot(self, geometries, output_path):
        """Save visualization screenshot"""
        vis = o3d.visualization.Visualizer()
        vis.create_window(visible=False)
        for geom in geometries:
            vis.add_geometry(geom)
        vis.update_renderer()
        vis.capture_screen_image(output_path)
        vis.destroy_window()
        print(f"Screenshot saved to: {output_path}")


def visualize_from_files(object_name, point_cloud_path, params_file, waypoints_file=None):
    """
    Load and visualize results from files
    
    Args:
        object_name: Object name
        point_cloud_path: Point cloud file path or list of paths
        params_file: Geometric parameters JSON file path
        waypoints_file: Trajectory JSON file path
    """
    visualizer = ResultVisualizer()
    
    # Load geometric parameters from JSON file
    params = {}
    if os.path.exists(params_file):
        with open(params_file, 'r', encoding='utf-8') as f:
            data = json.load(f)
            params = data.get('parameters', {})
    else:
        print(f"Warning: Parameters file not found: {params_file}")
    
    # Load trajectory
    trajectory = None
    if waypoints_file and os.path.exists(waypoints_file):
        with open(waypoints_file, 'r', encoding='utf-8') as f:
            data = json.load(f)
            trajectory = data.get('waypoints')
    
    # Visualize (pass parameter file path to load visualization function)
    visualizer.visualize_single_result(object_name, point_cloud_path, params, trajectory, params_file)


def visualize_all_from_files():
    """
    Automatically discover and visualize all objects from files
    
    Scans geometric_parameters, trajectories, and point_cloud directories
    to find all available objects and visualize them together.
    """
    print("\n=== Auto-discovering objects for visualization ===")
    
    # Define directories
    point_cloud_dir = "point_cloud"
    params_dir = "geometric_parameters"
    trajectory_dir = "trajectories"
    
    # Discover objects from params files
    objects_info = {}
    
    if os.path.exists(params_dir):
        for filename in os.listdir(params_dir):
            if filename.endswith('_params.json'):
                object_name = filename.replace('_params.json', '')
                
                # Load params file to get point cloud paths
                params_file = os.path.join(params_dir, filename)
                with open(params_file, 'r', encoding='utf-8') as f:
                    data = json.load(f)
                    point_cloud_paths = data.get('point_cloud_paths', [])
                    params = data.get('parameters', {})
                
                # Find corresponding trajectory file
                waypoints_file = os.path.join(trajectory_dir, f"{object_name}_waypoints.json")
                trajectory = None
                if os.path.exists(waypoints_file):
                    with open(waypoints_file, 'r', encoding='utf-8') as f:
                        traj_data = json.load(f)
                        trajectory = traj_data.get('waypoints')
                
                # Store info
                objects_info[object_name] = {
                    'parameterization': {
                        'point_cloud_paths': point_cloud_paths,
                        'parameters': params
                    },
                    'trajectory': {
                        'trajectory': trajectory
                    }
                }
                
                print(f"  ✓ Found object: {object_name}")
                print(f"    - Point clouds: {point_cloud_paths}")
                print(f"    - Parameters: {len(params)} components")
                print(f"    - Trajectory: {len(trajectory) if trajectory else 0} waypoints")
    
    if not objects_info:
        print("  ✗ No objects found!")
        return
    
    print(f"\nTotal objects found: {len(objects_info)}")
    
    # Visualize all objects
    visualizer = ResultVisualizer()
    visualizer.visualize_all_results(objects_info, point_cloud_dir, params_dir)


if __name__ == "__main__":
    # Test visualization
    print("Testing visualization module...")
    
    # Visualize all objects from files
    print("\n" + "="*60)
    print("Visualizing All Objects")
    print("="*60)
    visualize_all_from_files()
    
    # Or visualize single object:
    
    
    visualize_from_files(
        object_name="teacup",
        point_cloud_path=["point_cloud/teacup.ply"],
        params_file="geometric_parameters/teacup_params.json",
        waypoints_file="trajectories/teacup_waypoints.json"
    )
    
    visualize_from_files(
        object_name="tomato",
        point_cloud_path=["point_cloud/tomato.ply"],
        params_file="geometric_parameters/tomato_params.json",
        waypoints_file="trajectories/tomato_waypoints.json"
    )
    
    visualize_from_files(
        object_name="laptop",
        point_cloud_path=["point_cloud/laptop.ply"],
        params_file="geometric_parameters/laptop_params.json",
        waypoints_file="trajectories/laptop_waypoints.json"
    )
    
    visualize_from_files(
        object_name="cookie_box",
        point_cloud_path=["point_cloud/cookie_box.ply"],
        params_file="geometric_parameters/cookie_box_params.json",
        waypoints_file="trajectories/cookie_box_waypoints.json"
    )
    
    visualize_from_files(
        object_name="bowl",
        point_cloud_path=["point_cloud/bowl.ply"],
        params_file="geometric_parameters/bowl_params.json",
        waypoints_file="trajectories/bowl_waypoints.json"
    )
