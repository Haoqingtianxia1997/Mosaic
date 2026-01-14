import numpy as np
import open3d as o3d
import os


def generate_teacup_pointcloud(num_points=5000, save_path=None):
    """Generate teacup point cloud - only visible outer surface"""
    points = []
    
    # Cup outer wall (cylinder outer surface)
    for _ in range(int(num_points * 0.5)):
        theta = np.random.uniform(0, 2 * np.pi)
        height = np.random.uniform(0, 0.08)
        radius = 0.03 + height * 0.002  # Slightly expanding upward
        x = radius * np.cos(theta)
        y = radius * np.sin(theta)
        z = height
        points.append([x, y, z])
    
    # Cup rim edge (including visible part of inner wall upper edge)
    for _ in range(int(num_points * 0.25)):
        theta = np.random.uniform(0, 2 * np.pi)
        # Outer wall edge
        radius = 0.0316
        x = radius * np.cos(theta)
        y = radius * np.sin(theta)
        z = 0.08
        points.append([x, y, z])
    
    # Visible part of cup inner wall (small section visible from top view)
    for _ in range(int(num_points * 0.1)):
        theta = np.random.uniform(0, 2 * np.pi)
        depth = np.random.uniform(0, 0.01)  # Only top 1cm visible
        inner_radius = 0.028 - depth * 0.002
        x = inner_radius * np.cos(theta)
        y = inner_radius * np.sin(theta)
        z = 0.08 - depth
        points.append([x, y, z])
    
    
    points = np.array(points)
    
    # Apply rotation (tilt the teacup by 30 degrees around X-axis and 20 degrees around Y-axis)
    angle_x = np.radians(30)  # Tilt around X-axis
    angle_y = np.radians(20)  # Tilt around Y-axis
    
    # Rotation matrix around X-axis
    Rx = np.array([
        [1, 0, 0],
        [0, np.cos(angle_x), -np.sin(angle_x)],
        [0, np.sin(angle_x), np.cos(angle_x)]
    ])
    
    # Rotation matrix around Y-axis
    Ry = np.array([
        [np.cos(angle_y), 0, np.sin(angle_y)],
        [0, 1, 0],
        [-np.sin(angle_y), 0, np.cos(angle_y)]
    ])
    
    # Combined rotation matrix
    R = Ry @ Rx
    
    # Apply rotation
    points = points @ R.T
    
    # Translate to a new position (move away from origin)
    translation = np.array([0.5, 0.3, 0.02])  # Offset in x, y, z
    points = points + translation
    
    # Save point cloud
    if save_path:
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)
        pcd.paint_uniform_color([0.8, 0.8, 0.9])  # Light blue
        o3d.io.write_point_cloud(save_path, pcd)
        print(f"Teacup point cloud saved to: {save_path}")
    
    return points


def generate_bowl_pointcloud(num_points=5000, save_path=None):
    """Generate bowl point cloud - only visible outer surface"""
    points = []
    
    # Bowl outer wall (hemisphere shape)
    for _ in range(int(num_points * 0.6)):
        theta = np.random.uniform(0, 2 * np.pi)
        # phi from 0 to about 75 degrees (upper portion of hemisphere)
        phi = np.random.uniform(0, np.radians(75))
        
        # Bowl dimensions
        base_radius = 0.08  # Bottom radius
        top_radius = 0.10   # Top opening radius
        height_factor = np.cos(phi)
        
        # Radius varies from top to bottom
        radius = top_radius - (top_radius - base_radius) * height_factor
        
        x = radius * np.sin(phi) * np.cos(theta)
        y = radius * np.sin(phi) * np.sin(theta)
        z = base_radius * np.cos(phi)
        
        # Add slight irregularity
        noise = np.random.normal(0, 0.001, 3)
        points.append([x + noise[0], y + noise[1], z + noise[2]])
    
    # Bowl rim (top edge)
    for _ in range(int(num_points * 0.25)):
        theta = np.random.uniform(0, 2 * np.pi)
        x = top_radius * np.cos(theta)
        y = top_radius * np.sin(theta)
        z = base_radius * np.cos(np.radians(75))
        points.append([x, y, z])
    
    # Visible inner wall (from top view)
    for _ in range(int(num_points * 0.15)):
        theta = np.random.uniform(0, 2 * np.pi)
        phi = np.random.uniform(0, np.radians(30))  # Only upper inner part visible
        
        inner_radius_scale = 0.95  # Inner surface slightly smaller
        radius = (top_radius - (top_radius - base_radius) * np.cos(phi)) * inner_radius_scale
        
        x = radius * np.sin(phi) * np.cos(theta)
        y = radius * np.sin(phi) * np.sin(theta)
        z = base_radius * np.cos(phi) - 0.005  # Slightly lower than outer wall
        
        points.append([x, y, z])
    
    points = np.array(points)
    
    # Apply rotation (slight tilt)
    angle_x = np.radians(5)
    angle_y = np.radians(180)
    
    Rx = np.array([
        [1, 0, 0],
        [0, np.cos(angle_x), -np.sin(angle_x)],
        [0, np.sin(angle_x), np.cos(angle_x)]
    ])
    
    Ry = np.array([
        [np.cos(angle_y), 0, np.sin(angle_y)],
        [0, 1, 0],
        [-np.sin(angle_y), 0, np.cos(angle_y)]
    ])
    
    R = Ry @ Rx
    points = points @ R.T
    
    # Translate to position
    translation = np.array([0.0, 0.0, 0.05])
    points = points + translation
    
    # Save point cloud
    if save_path:
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)
        pcd.paint_uniform_color([0.9, 0.9, 0.85])  # Light cream color
        o3d.io.write_point_cloud(save_path, pcd)
        print(f"Bowl point cloud saved to: {save_path}")
    
    return points


def generate_tomato_pointcloud(num_points=5000, save_path=None):
    """Generate tomato point cloud - only visible outer surface, returns points and colors"""
    points = []
    colors = []
    
    # Main body outer surface (ellipsoid, slightly flattened, only upper half and visible side parts)
    for _ in range(int(num_points * 0.85)):
        theta = np.random.uniform(0, 2 * np.pi)
        # phi only takes upper hemisphere and sides, excluding bottom touching ground
        phi = np.random.uniform(0, np.pi * 0.85)  # Exclude bottom 15%
        
        # Use ellipsoid
        radius_xy = 0.04
        radius_z = 0.035
        
        x = radius_xy * np.sin(phi) * np.cos(theta)
        y = radius_xy * np.sin(phi) * np.sin(theta)
        z = radius_z * np.cos(phi)
        
        # Add some irregularity
        noise = np.random.normal(0, 0.002, 3)
        points.append([x + noise[0], y + noise[1], z + noise[2]])
        colors.append([0.9, 0.2, 0.2])  # Red
    
    # Top calyx (green, petal shape)
    num_petals = 5  # 5 calyx petals
    for _ in range(int(num_points * 0.3)):
        # Select a petal
        petal_idx = np.random.randint(0, num_petals)
        petal_angle = petal_idx * (2 * np.pi / num_petals)
        
        # Random position within petal
        petal_spread = np.random.uniform(0, 0.6)  # Petal width
        radial_pos = np.random.uniform(0.005, 0.018)  # Distance from center outward
        
        # Calculate position
        angle = petal_angle + (petal_spread - 0.3) * 0.5
        height = np.random.uniform(0, 0.008)
        
        x = radial_pos * np.cos(angle)
        y = radial_pos * np.sin(angle)
        z = 0.035 + height - radial_pos * 0.15  # Calyx slopes downward
        
        points.append([x, y, z])
        # Green gradient, darker edges
        green_intensity = 0.5 + 0.3 * (1 - radial_pos / 0.018)
        colors.append([0.2, green_intensity, 0.15])
    
    # Central small stem
    for _ in range(int(num_points * 0.05)):
        theta = np.random.uniform(0, 2 * np.pi)
        height = np.random.uniform(0.008, 0.015)
        radius = 0.003 * (1 - (height - 0.008) / 0.007)
        
        x = radius * np.cos(theta)
        y = radius * np.sin(theta)
        z = 0.035 + height
        points.append([x, y, z])
        colors.append([0.15, 0.5, 0.1])  # Dark green
    
    points = np.array(points)
    colors = np.array(colors)
    
    # Save point cloud
    if save_path:
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)
        pcd.colors = o3d.utility.Vector3dVector(colors)
        o3d.io.write_point_cloud(save_path, pcd)
        print(f"Tomato point cloud saved to: {save_path}")
    
    return points, colors


def generate_cookie_box_pointcloud(num_points=5000, save_path=None):
    """Generate cookie box point cloud - only visible outer surface"""
    points = []
    
    # Box dimensions
    width = 0.15
    depth = 0.10
    height = 0.08
    
    # Front and back faces (outer surface)
    for _ in range(int(num_points * 0.3)):
        x = np.random.uniform(-width/2, width/2)
        z = np.random.uniform(0, height)
        y_face = depth/2 if np.random.random() > 0.5 else -depth/2
        points.append([x, y_face, z])
    
    # Left and right faces (outer surface)
    for _ in range(int(num_points * 0.3)):
        y = np.random.uniform(-depth/2, depth/2)
        z = np.random.uniform(0, height)
        x_face = width/2 if np.random.random() > 0.5 else -width/2
        points.append([x_face, y, z])
    
    # Top face (visible)
    for _ in range(int(num_points * 0.4)):
        x = np.random.uniform(-width/2, width/2)
        y = np.random.uniform(-depth/2, depth/2)
        points.append([x, y, height])
    
    # Not generating bottom face - blocked by table
    
    points = np.array(points)
    
    # Save point cloud
    if save_path:
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)
        pcd.paint_uniform_color([0.8, 0.6, 0.2])  # Golden yellow
        o3d.io.write_point_cloud(save_path, pcd)
        print(f"Cookie box point cloud saved to: {save_path}")
    
    return points


def generate_laptop_pointcloud(num_points=8000, save_path=None):
    """Generate laptop point cloud - only visible outer surface"""
    base_points = []
    screen_points = []
    
    # Keyboard base part
    base_width = 0.30
    base_depth = 0.20
    base_height = 0.02
    
    # Base top surface (keyboard surface - visible)
    for _ in range(int(num_points * 0.4)):
        x = np.random.uniform(-base_width/2, base_width/2)
        y = np.random.uniform(-base_depth/2, base_depth/2)
        base_points.append([x, y, base_height])
    
    # Base front face (facing user - visible)
    for _ in range(int(num_points * 0.1)):
        x = np.random.uniform(-base_width/2, base_width/2)
        z = np.random.uniform(0, base_height)
        base_points.append([x, base_depth/2, z])
    
    # Base left and right side faces (partially visible)
    for _ in range(int(num_points * 0.05)):
        y = np.random.uniform(-base_depth/2, base_depth/2)
        z = np.random.uniform(0, base_height)
        x_face = base_width/2 if np.random.random() > 0.5 else -base_width/2
        base_points.append([x_face, y, z])
    
    # Not generating base bottom and back - bottom blocked by table, back blocked by screen
    
    # Screen part (opened about 120 degrees)
    screen_width = 0.30
    screen_height = 0.20
    screen_thickness = 0.01
    
    # Screen angle
    angle = np.radians(30)  # Tilted 30 degrees from vertical
    
    # Screen front (display - facing user, visible)
    for _ in range(int(num_points * 0.35)):
        x = np.random.uniform(-screen_width/2, screen_width/2)
        screen_y = np.random.uniform(0, screen_height)
        
        # Rotation transformation
        y = -base_depth/2 + screen_y * np.cos(angle)
        z = base_height + screen_y * np.sin(angle)
        screen_points.append([x, y, z])
    
    # Screen back (outer shell - facing away from user, partially visible)
    for _ in range(int(num_points * 0.05)):
        x = np.random.uniform(-screen_width/2, screen_width/2)
        screen_y = np.random.uniform(0, screen_height)
        
        y = -base_depth/2 + screen_y * np.cos(angle) - screen_thickness * np.sin(angle)
        z = base_height + screen_y * np.sin(angle) - screen_thickness * np.cos(angle)
        screen_points.append([x, y, z])
    
    base_points = np.array(base_points)
    screen_points = np.array(screen_points)
    all_points = np.vstack([base_points, screen_points])
    
    # Save point clouds
    if save_path:
        # Save complete laptop
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(all_points)
        pcd.paint_uniform_color([0.3, 0.3, 0.3])  # Dark gray
        o3d.io.write_point_cloud(save_path, pcd)
        print(f"Laptop point cloud saved to: {save_path}")
        
        # Save base separately
        base_path = save_path.replace('.ply', '_base.ply')
        pcd_base = o3d.geometry.PointCloud()
        pcd_base.points = o3d.utility.Vector3dVector(base_points)
        pcd_base.paint_uniform_color([0.3, 0.3, 0.3])  # Dark gray
        o3d.io.write_point_cloud(base_path, pcd_base)
        print(f"Laptop base point cloud saved to: {base_path}")
        
        # Save screen separately
        screen_path = save_path.replace('.ply', '_screen.ply')
        pcd_screen = o3d.geometry.PointCloud()
        pcd_screen.points = o3d.utility.Vector3dVector(screen_points)
        pcd_screen.paint_uniform_color([0.2, 0.2, 0.2])  # Slightly darker gray
        o3d.io.write_point_cloud(screen_path, pcd_screen)
        print(f"Laptop screen point cloud saved to: {screen_path}")
    
    return all_points, base_points, screen_points


def generate_all_pointclouds():
    """Generate all point clouds and save to point_cloud folder"""
    
    # Create save directory
    save_dir = "point_cloud"
    os.makedirs(save_dir, exist_ok=True)
    
    # Generate and save point clouds - reduced point count to make them sparser
    print("Generating point clouds...")
    teacup_points = generate_teacup_pointcloud(
        num_points=1500, 
        save_path=os.path.join(save_dir, "teacup.ply")
    )
    
    bowl_points = generate_bowl_pointcloud(
        num_points=1500,
        save_path=os.path.join(save_dir, "bowl.ply")
    )
    
    tomato_points, tomato_colors = generate_tomato_pointcloud(
        num_points=1500,
        save_path=os.path.join(save_dir, "tomato.ply")
    )
    
    cookie_box_points = generate_cookie_box_pointcloud(
        num_points=1200,
        save_path=os.path.join(save_dir, "cookie_box.ply")
    )
    
    laptop_all, laptop_base, laptop_screen = generate_laptop_pointcloud(
        num_points=2000,
        save_path=os.path.join(save_dir, "laptop.ply")
    )
    
    print(f"\nAll point clouds generated and saved to '{save_dir}' folder")
    print(f"  - Laptop saved as 3 files: laptop.ply (complete), laptop_base.ply, laptop_screen.ply")


def visualize_pointclouds():
    """Load and visualize all point clouds"""
    
    save_dir = "point_cloud"
    
    # Check if files exist
    if not os.path.exists(save_dir):
        print(f"Error: '{save_dir}' folder does not exist, please run generate_all_pointclouds() first")
        return
    
    # Load point cloud files
    print("Loading point cloud files...")
    pcd_teacup = o3d.io.read_point_cloud(os.path.join(save_dir, "teacup.ply"))
    pcd_bowl = o3d.io.read_point_cloud(os.path.join(save_dir, "bowl.ply"))
    pcd_tomato = o3d.io.read_point_cloud(os.path.join(save_dir, "tomato.ply"))
    pcd_cookie_box = o3d.io.read_point_cloud(os.path.join(save_dir, "cookie_box.ply"))
    pcd_laptop_base = o3d.io.read_point_cloud(os.path.join(save_dir, "laptop_base.ply"))
    pcd_laptop_screen = o3d.io.read_point_cloud(os.path.join(save_dir, "laptop_screen.ply"))
    
    # Adjust positions to avoid overlap
    teacup_points = np.asarray(pcd_teacup.points)
    teacup_points[:, 0] += -0.5
    pcd_teacup.points = o3d.utility.Vector3dVector(teacup_points)
    
    bowl_points = np.asarray(pcd_bowl.points)
    bowl_points[:, 0] += -0.25
    pcd_bowl.points = o3d.utility.Vector3dVector(bowl_points)
    
    tomato_points = np.asarray(pcd_tomato.points)
    tomato_points[:, 0] += 0.0
    pcd_tomato.points = o3d.utility.Vector3dVector(tomato_points)
    
    cookie_box_points = np.asarray(pcd_cookie_box.points)
    cookie_box_points[:, 0] += 0.25
    pcd_cookie_box.points = o3d.utility.Vector3dVector(cookie_box_points)
    
    # Position laptop base
    laptop_base_points = np.asarray(pcd_laptop_base.points)
    laptop_base_points[:, 0] += 0.6
    pcd_laptop_base.points = o3d.utility.Vector3dVector(laptop_base_points)
    pcd_laptop_base.paint_uniform_color([0.4, 0.4, 0.4])  # Light gray for base
    
    # Position laptop screen (same x position as base)
    laptop_screen_points = np.asarray(pcd_laptop_screen.points)
    laptop_screen_points[:, 0] += 0.6
    pcd_laptop_screen.points = o3d.utility.Vector3dVector(laptop_screen_points)
    pcd_laptop_screen.paint_uniform_color([0.1, 0.3, 0.5])  # Blue-gray for screen
    
    print("Visualizing point clouds...")
    print("  - 6 objects: teacup, bowl, tomato, cookie_box, laptop_base (gray), laptop_screen (blue-gray)")
    # Visualize with separate base and screen
    o3d.visualization.draw_geometries(
        [pcd_teacup, pcd_bowl, pcd_tomato, pcd_cookie_box, pcd_laptop_base, pcd_laptop_screen],
        window_name="6 Objects: Teacup, Bowl, Tomato, Cookie Box, Laptop Base, Laptop Screen",
        width=1400,
        height=800
    )


if __name__ == "__main__":
    # First generate and save point clouds
    generate_all_pointclouds()
    
    # Then load and visualize
    visualize_pointclouds()
