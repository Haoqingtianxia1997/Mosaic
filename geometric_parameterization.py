# coding: utf-8
"""
Geometric Parameterization Module - Convert point clouds to geometric parameters using LLM

Workflow: Input point cloud → LLM selects primitives → Generate algorithm+visualization → Execute and extract parameters → Save separately
"""
import numpy as np
import open3d as o3d
import os
import json
from llm_interface import LLMInterface


class GeometricParameterizer:
    """Geometric Parameterizer - LLM automatically selects primitives and generates visualization"""
    
    # Available primitive types and descriptions
    AVAILABLE_PRIMITIVES = {
        'box': 'Box - 6-faced rectangular prism, Params: center, dimensions [width, depth, height], rotation_matrix (3x3, optional for oriented box)',
        'cylinder': 'Cylinder - circular-base cylinder with closed top/bottom faces, Params: center, radius, height, axis (direction vector from bottom to top)',
        'sphere': 'Sphere - complete closed sphere, Params: center, radius',
        'hemisphere': 'Hemisphere - half sphere with flat circular base, Params: center, radius, axis (normal vector pointing FROM curved dome CENTER TOWARD the flat opening, i.e., the direction you look to see inside)',
        'cone': 'Cone - circular base to apex with closed base, Params: base_center, base_radius, height, axis (direction vector from base to apex)',
        'pyramid': 'Pyramid - polygonal base to apex with closed base, Params: base_center, base_vertices, height, axis (direction vector from base to apex)',
        'frustum_cone': 'Conical Frustum - truncated cone with closed top/bottom, Params: bottom_center, bottom_radius, top_center, top_radius, height',
        'frustum_pyramid': 'Pyramidal Frustum - truncated pyramid with closed top/bottom, Params: bottom_center, bottom_vertices, top_center, top_vertices, height',
        'torus': 'Torus - closed donut shape, Params: center, major_radius, minor_radius, axis (normal to torus plane)',
        'ellipsoid': 'Ellipsoid - closed ellipsoid with three different semi-axes, Params: center, radii [rx, ry, rz], rotation_matrix (3x3, optional)',
    }
    
    def __init__(self, llm_interface):
        self.llm = llm_interface
        self.system_prompt = self._create_system_prompt()
    
    def _create_system_prompt(self):
        """Create system prompt"""
        return """You are an expert in 3D geometry analysis and computer graphics. 

Your task:
1. Analyze point cloud data and select appropriate geometric primitives
2. Generate algorithms to extract parameters from the point cloud
3. Generate visualization functions to display the fitted geometry

Available primitives and their typical use cases will be provided in the prompt.

CRITICAL REQUIREMENTS:
1. **TYPE CONSISTENCY**: The 'type' field in parameters MUST match the selected primitive name EXACTLY!
   - If selected primitive is 'cylinder', type MUST be 'cylinder' (not 'box')
   - If selected primitive is 'box', type MUST be 'box' (not 'cylinder')
2. The extracted geometric primitives MUST enclose/envelope the entire point cloud！！！！ Must ensure full enclosure
3. Use ACTUAL min/max bounds from the point cloud to ensure enclosure
4. For composite objects, decompose into multiple primitives with proper segmentation
5. Each component needs both parameters AND a visualization function
6. All visualization functions should use Open3D mesh creation
7. **HANDLE ORIENTATION INTELLIGENTLY**: 
   - For naturally axis-aligned objects (upright cylinders, axis-parallel boxes), use simple geometry WITHOUT rotation
   - ONLY use PCA and rotation matrix for clearly tilted/rotated objects (like laptop screen)
   - Check alignment with world axes before deciding to use rotation
8. The center of the geometry should be based on the actual center position of the point cloud, not the origin

Return ONLY executable Python code, no markdown formatting, no explanations."""
    
    def select_primitives_with_llm(self, object_name, points, subtask_semantic=None):
        """Use LLM to automatically select primitives based on object semantics"""
        # Statistics
        num_points = len(points)
        min_bounds = np.min(points, axis=0)
        max_bounds = np.max(points, axis=0)
        dimensions = max_bounds - min_bounds
        centroid = np.mean(points, axis=0)
        
        # Build primitive selection prompt
        primitives_desc = "Available Geometric Primitives:\n\n"
        for idx, (prim_name, prim_desc) in enumerate(self.AVAILABLE_PRIMITIVES.items(), 1):
            primitives_desc += f"{idx}. **{prim_name}**: {prim_desc}\n"
        
        prompt = f"""Analyze this object and select appropriate geometric primitives for fitting.

Object: {object_name}
{f"Task Context: {subtask_semantic}" if subtask_semantic else ""}

Point Cloud Statistics:
- Total points: {num_points}
- Spatial dimensions: [{dimensions[0]:.4f}, {dimensions[1]:.4f}, {dimensions[2]:.4f}] meters
- Bounding box: [{min_bounds[0]:.4f}, {min_bounds[1]:.4f}, {min_bounds[2]:.4f}] to [{max_bounds[0]:.4f}, {max_bounds[1]:.4f}, {max_bounds[2]:.4f}]

{primitives_desc}

Based on the object name "{object_name}" and its dimensions, select one most appropriate primitive.

Examples:
- "teacup" or "cup" → cylinder 
- "tomato" → ellipsoid 
- "cookie_box" or "laptop" → box
- "bowl" → hemisphere
- "cone" → cone
- "table" → box (top)

Return ONLY a Python list of primitive names, for example:
['cylinder', 'torus']

or for simple objects:
['ellipsoid']

Return ONLY the list, nothing else."""

        response = self.llm.generate_code(prompt, "", temperature=0.3)
        
        # Parse response and extract list
        try:
            # Try direct eval
            import ast
            response_clean = response.strip()
            if response_clean.startswith('[') and response_clean.endswith(']'):
                selected = ast.literal_eval(response_clean)
            else:
                # Try to find list
                import re
                match = re.search(r'\[.*?\]', response, re.DOTALL)
                if match:
                    selected = ast.literal_eval(match.group())
                else:
                    # Default fallback
                    print(f"  Warning: Unable to parse LLM response, using default primitives")
                    selected = ['box']
            
            # Verify selected primitives are valid
            selected = [p for p in selected if p in self.AVAILABLE_PRIMITIVES]
            if not selected:
                selected = ['box']
                
            print(f"  LLM auto-selected primitives: {selected}")
            return selected
            
        except Exception as e:
            print(f"  Warning: Failed to parse primitive selection ({e}), using defaults")
            return ['box']
    
    def assemble_prompt(self, object_name, points_dict, primitive_names_dict, subtask_semantic=None):
        """Assemble prompt: subtask + point cloud data + primitive specs + generation requirements
        
        Args:
            object_name: Object name
            points_dict: {part_name: points_array} or single points_array
            primitive_names_dict: {part_name: [primitives]} or single [primitives]
            subtask_semantic: Task description
        """
        # Normalize input: convert to dictionary format
        if isinstance(points_dict, np.ndarray):
            points_dict = {'component': points_dict}
            primitive_names_dict = {'component': primitive_names_dict}
        
        # 1. Task description
        if subtask_semantic:
            task_desc = f"Subtask: {subtask_semantic}\n"
        else:
            task_desc = f"Task: Geometric parameterization and visualization\n"
        task_desc += f"Object: {object_name}\n"
        
        # 2. Point cloud statistics (for each part)
        cloud_info = "\nPoint Cloud Data:\n"
        if len(points_dict) == 1:
            part_name, points = list(points_dict.items())[0]
            num_points = len(points)
            min_bounds = np.min(points, axis=0)
            max_bounds = np.max(points, axis=0)
            dimensions = max_bounds - min_bounds
            centroid = np.mean(points, axis=0)
            
            cloud_info += f"""The function will receive a numpy array `points` with shape ({num_points}, 3).

Statistics:
- Total points: {num_points}
- Dimensions: [{dimensions[0]:.4f}, {dimensions[1]:.4f}, {dimensions[2]:.4f}] meters
- Center: [{centroid[0]:.4f}, {centroid[1]:.4f}, {centroid[2]:.4f}]
- Bounds: [{min_bounds[0]:.4f}, {min_bounds[1]:.4f}, {min_bounds[2]:.4f}] to [{max_bounds[0]:.4f}, {max_bounds[1]:.4f}, {max_bounds[2]:.4f}]
"""
        else:
            cloud_info += f"The function will receive a dictionary `points_dict` with {len(points_dict)} parts:\n\n"
            for part_name, points in points_dict.items():
                num_points = len(points)
                min_bounds = np.min(points, axis=0)
                max_bounds = np.max(points, axis=0)
                dimensions = max_bounds - min_bounds
                centroid = np.mean(points, axis=0)
                
                cloud_info += f"""Part '{part_name}': shape ({num_points}, 3)
- Points: {num_points}
- Dimensions: [{dimensions[0]:.4f}, {dimensions[1]:.4f}, {dimensions[2]:.4f}] meters
- Center: [{centroid[0]:.4f}, {centroid[1]:.4f}, {centroid[2]:.4f}]
- Bounds: [{min_bounds[0]:.4f}, {min_bounds[1]:.4f}, {min_bounds[2]:.4f}] to [{max_bounds[0]:.4f}, {max_bounds[1]:.4f}, {max_bounds[2]:.4f}]

"""
        
        # 3. Selected primitives description
        primitives_info = "\nSelected Primitives:\n"
        if len(points_dict) == 1:
            primitive_names = list(primitive_names_dict.values())[0]
            for prim_name in primitive_names:
                if prim_name in self.AVAILABLE_PRIMITIVES:
                    primitives_info += f"- {prim_name}: {self.AVAILABLE_PRIMITIVES[prim_name]}\n"
        else:
            for part_name, primitive_names in primitive_names_dict.items():
                primitives_info += f"\nPart '{part_name}':\n"
                for prim_name in primitive_names:
                    if prim_name in self.AVAILABLE_PRIMITIVES:
                        primitives_info += f"  - {prim_name}: {self.AVAILABLE_PRIMITIVES[prim_name]}\n"
        
        # 4. Generation requirements
        if len(points_dict) == 1:
            part_name, points = list(points_dict.items())[0]
            min_bounds = np.min(points, axis=0)
            max_bounds = np.max(points, axis=0)
            input_desc = "numpy array (N, 3)"
            param_example = """{{
         'parameters': {{
             'component_0': {{'type': 'primitive_name', 'param1': value1, ...}},
             'component_1': {{'type': 'primitive_name', 'param2': value2, ...}},
             ...
         }}
     }}"""
            bounds_hint = f"min=[{min_bounds[0]:.4f}, {min_bounds[1]:.4f}, {min_bounds[2]:.4f}], max=[{max_bounds[0]:.4f}, {max_bounds[1]:.4f}, {max_bounds[2]:.4f}]"
        else:
            input_desc = "dictionary with structure {{'part1': points_array1, 'part2': points_array2, ...}}"
            param_example = """{{
         'parameters': {{
             'part1_component_0': {{'type': 'primitive_name', 'param1': value1, ...}},
             'part2_component_0': {{'type': 'primitive_name', 'param1': value1, ...}},
             ...
         }}
     }}"""
            bounds_hint = "Use actual bounds from each part's point cloud"
        
        requirements = f"""
YOUR TASK: Generate TWO functions

1. **extract_parameters(points)** - Extract geometric parameters
   - Input: {input_desc}
   - Output: Dictionary with structure:
     {param_example}
   - CRITICAL: Primitives MUST enclose entire point cloud
   - Use actual bounds: {bounds_hint}
   - **MANDATORY PCA CHECK FOR ALL PRIMITIVES**: For EVERY primitive (box, cylinder, ellipsoid, etc.), MUST perform PCA to determine orientation:
     ```python
     # Step 1: Compute bounds and initial center
     min_bounds = np.min(points, axis=0)
     max_bounds = np.max(points, axis=0)
     center = (min_bounds + max_bounds) / 2
     
     # Step 2: ALWAYS compute PCA to find principal axes
     centered = points - center
     cov = np.cov(centered.T)
     eigenvalues, eigenvectors = np.linalg.eig(cov)
     idx = eigenvalues.argsort()[::-1]
     rotation_matrix = eigenvectors[:, idx]
     if np.linalg.det(rotation_matrix) < 0:
         rotation_matrix[:, 2] *= -1
     
     # Step 3: Check alignment with world axes (ALL three axes)
     threshold = 0.95
     x_aligned = max(abs(rotation_matrix[0, 0]), abs(rotation_matrix[1, 0]), abs(rotation_matrix[2, 0])) > threshold
     y_aligned = max(abs(rotation_matrix[0, 1]), abs(rotation_matrix[1, 1]), abs(rotation_matrix[2, 1])) > threshold
     z_aligned = max(abs(rotation_matrix[0, 2]), abs(rotation_matrix[1, 2]), abs(rotation_matrix[2, 2])) > threshold
     is_axis_aligned = x_aligned and y_aligned and z_aligned
     
     # Step 4: Based on alignment, compute parameters
     if not is_axis_aligned:
         # Object is tilted/rotated - use oriented primitive
         # Transform to local frame for dimension calculation
         local_points = centered @ rotation_matrix
         # For box/ellipsoid: compute dimensions/radii in local frame
         # For cylinder: compute axis direction from rotation_matrix
         # MUST store rotation_matrix or axis in parameters
     else:
         # Object is axis-aligned - use simple primitive
         # Use global frame dimensions
         # Do NOT store rotation_matrix (optional for backward compatibility)
     ```
   - **PARAMETER CALCULATION RULES**:
     * **Box**: dimensions from local frame if rotated, global frame if aligned
     * **Cylinder**: CRITICAL - use FIRST principal component (longest dimension) as cylinder axis!
       - The FIRST principal component (rotation_matrix[:, 0]) represents the HEIGHT direction
       - DO NOT use rotation_matrix[:, 2] - that's for naturally Z-aligned objects!
       - Check alignment: use dot product with [0,0,1]: `abs(np.dot(axis, [0,0,1])) > 0.95`
       - If rotated, store axis direction vector (from bottom to top); if aligned (Z-up), no axis needed
       - Oriented: axis=rotation_matrix[:,0].tolist(), radius, height, center
       - Aligned: radius, height, center (no axis)
     * **Hemisphere**: CRITICAL - axis direction AND center position
       - **NEW CLEAR DEFINITION**: axis = normal vector pointing FROM curved dome CENTER TOWARD the flat opening
         * Think of it as "the direction you look to see inside the hemisphere"
         * For BOWL (opening up, like a cup): dome at bottom, opening at top → axis points UP [0,0,1]
         * For DOME (opening down, like umbrella): dome at top, opening at bottom → axis points DOWN [0,0,-1]
       - **CENTER POSITION IS CRITICAL**: The 'center' should be at the CENTER OF THE SPHERE (not at the flat base!)
         * For a hemisphere, the geometric center is at the center of the full sphere it's cut from
         * This is NOT the centroid of the point cloud!
         * Calculate: center = centroid of actual points, then adjust along axis direction if needed
       - **ROBUST Direction determination** - Use CURVATURE analysis, not just point distribution:
         ```python
         # Analyze point cloud shape to determine opening direction
         min_bounds = np.min(points, axis=0)
         max_bounds = np.max(points, axis=0)
         center = (min_bounds + max_bounds) / 2
         
         # Key insight: The OPENING side has LARGER spread (wider radius)
         # The DOME side has SMALLER spread (pointy/curved end)
         
         # Divide point cloud into top and bottom halves
         centered = points - center
         top_points = points[centered[:, 2] > 0]  # Upper half
         bottom_points = points[centered[:, 2] < 0]  # Lower half
         
         # Calculate XY spread (radius) for each half
         if len(top_points) > 10 and len(bottom_points) > 10:
             top_center_xy = np.mean(top_points[:, :2], axis=0)
             bottom_center_xy = np.mean(bottom_points[:, :2], axis=0)
             
             top_radius = np.max(np.linalg.norm(top_points[:, :2] - top_center_xy, axis=1))
             bottom_radius = np.max(np.linalg.norm(bottom_points[:, :2] - bottom_center_xy, axis=1))
             
             # If top has larger radius → opening is at top (bowl)
             # If bottom has larger radius → opening is at bottom (dome/inverted bowl)
             opening_at_top = top_radius > bottom_radius * 1.1  # 10% threshold for robustness
         else:
             # Fallback: use point density
             opening_at_top = len(top_points) > len(bottom_points)
         
         if opening_at_top:
             # Bowl: opening at top, dome at bottom, axis points UP
             axis = np.array([0, 0, 1])
             # Center at opening level (top)
             center[2] = max_bounds[2]
         else:
             # Dome/inverted: opening at bottom, dome at top, axis points DOWN
             axis = np.array([0, 0, -1])
             # Center at opening level (bottom)
             center[2] = min_bounds[2]
         
         # Calculate radius
         distances = np.linalg.norm(points - center, axis=1)
         radius = np.max(distances) * 1.02
         ```
       - Oriented: axis (direction from dome to opening), radius, center
       - Aligned: axis=[0,0,1] for bowl or [0,0,-1] for dome, radius, center
     * **Ellipsoid**: radii from max extents along principal axes (NOT variance!)
       - Compute: `radii = [np.max(np.abs(local_points[:, i])) for i in range(3)]`
       - Oriented: radii, rotation_matrix, center
       - Aligned: radii, center (no rotation_matrix)
     * **Sphere**: radius from max distance to center
     * **Cone/Pyramid**: axis direction from base to apex, use PCA to determine orientation
   - **ADD MARGIN**: Multiply dimensions/radii by 1.01-1.02 to ensure complete enclosure
   - **PRIMITIVE TYPE CONSISTENCY**: The 'type' field MUST match the selected primitive name exactly!
   - For composite objects: use DBSCAN/KMeans to segment first
   - All values must be JSON-serializable (use .tolist() for arrays)

2. **create_visualization_geometry(params)** - Create Open3D meshes for visualization
   - Input: params dict (same structure as extract_parameters output)
   - Output: List of Open3D geometry objects (meshes, wireframes)
   - For EACH component in params['parameters'], create corresponding mesh
   - **MATCH THE PRIMITIVE TYPE**: Check comp_params['type'] and create the correct geometry!
   
   CRITICAL: Open3D geometries have LOCAL coordinate systems! Transform correctly:
   
   **BOX**: create_box(width, height, depth)
   - Local origin: Corner at [0, 0, 0], extends to [width, height, depth]
   - If rotation_matrix exists (oriented box):
     1. Create: `mesh = o3d.geometry.TriangleMesh.create_box(width=dims[0], height=dims[1], depth=dims[2])`
     2. Center: `mesh.translate(-dims/2)` to move corner to center
     3. Rotate: `mesh.rotate(rotation_matrix, center=[0,0,0])`
     4. Position: `mesh.translate(center)`
   - If no rotation_matrix (axis-aligned):
     1. Create: `mesh = o3d.geometry.TriangleMesh.create_box(width=dims[0], height=dims[1], depth=dims[2])`
     2. Position: `mesh.translate(center - dims/2)` to move corner to center
   
   **CYLINDER**: create_cylinder(radius, height)
   - Local origin: **CENTER at [0, 0, 0]**, axis along Z from [0,0,-height/2] to [0,0,+height/2]
   - Already centered! Just translate to final position
   - If axis provided (oriented cylinder):
     1. Create Z-aligned: `mesh = o3d.geometry.TriangleMesh.create_cylinder(radius=r, height=h)`
     2. Compute rotation from [0,0,1] to axis direction
     3. Rotate at origin: `mesh.rotate(R, center=[0,0,0])`
     4. Translate to center: `mesh.translate(center)`
   - If no axis (Z-aligned):
     1. Create: `mesh = o3d.geometry.TriangleMesh.create_cylinder(radius=r, height=h)`
     2. **Direct translate**: `mesh.translate(center)` (already centered!)
   
   **SPHERE**: create_sphere(radius)
   - Local origin: **CENTER at [0, 0, 0]**
   - Already centered! Just translate to final position
   - Create: `mesh = o3d.geometry.TriangleMesh.create_sphere(radius=r)`
   - Transform: `mesh.translate(center)` (already centered!)
   
   **ELLIPSOID** (scaled sphere):
   - Local origin: **CENTER at [0, 0, 0]**
   - Already centered! Scale then translate
   - Parameters: center, radii [rx, ry, rz], rotation_matrix (3x3, optional)
   - Steps:
     1. Create unit sphere: `mesh = o3d.geometry.TriangleMesh.create_sphere(radius=1.0)`
     2. Scale each axis: Apply scaling for rx, ry, rz
     3. Apply rotation if provided: `mesh.rotate(rotation_matrix, center=[0,0,0])`
     4. Translate: `mesh.translate(center)` (already centered!)
   
   **CONE**: create_cone(radius, height)
   - Local origin: **CENTER at [0, 0, 0]**, axis along Z, base at [0,0,-height/2], apex at [0,0,+height/2]
   - Already centered! Similar to cylinder
   - Rotate if needed, then translate to center
   
   **HEMISPHERE**: create_sphere(radius) then cut
   - CRITICAL: Hemisphere = half of sphere with FLAT OPENING and CURVED DOME
   - **CRITICAL CENTER POSITION**: Open3D creates sphere centered at [0,0,0]
     * When you cut the sphere to make hemisphere, the center STAYS at the sphere center
     * For a hemisphere with flat opening on XY plane at z=0, the sphere center is AT z=0 (on the opening plane)
     * The curved dome extends in the +Z or -Z direction depending on which half you keep
   - **REVISED axis DEFINITION**: axis = normal vector pointing FROM curved dome CENTER TOWARD the flat opening
     * This is the direction you look to see inside the hemisphere
     * Default: axis=[0,0,1] means dome below (at negative Z), opening above (at z=0), points upward
   - **The 'center' parameter in your params**: This is the SPHERE CENTER (at the opening plane), NOT offset!
   - **Direction convention** (CORRECTED):
     * For BOWL (opening up): dome at bottom, opening at top → axis points UP [0,0,1]
       The sphere center should be at the RIM level (top/opening of bowl)
       Keep vertices where axis · vertex < 0 (dome side, negative projection)
     * For DOME (opening down): dome at top, opening at bottom → axis points DOWN [0,0,-1]
       The sphere center should be at the BASE level (bottom/opening of dome)
       Keep vertices where axis · vertex < 0 (dome side, negative projection)
   - **Proper implementation** (CORRECTED):
     1. Create full sphere: `mesh = o3d.geometry.TriangleMesh.create_sphere(radius=r, resolution=20)`
     2. Get vertices and triangles as numpy arrays
     3. Determine cutting plane based on axis direction:
        ```python
        axis = np.array(comp_params.get('axis', [0, 0, 1]))  # Default: bowl
        axis = axis / np.linalg.norm(axis)
        
        # CRITICAL: axis points FROM dome TO opening
        # We want to KEEP the dome side, which is OPPOSITE to axis direction
        # Project vertices onto axis: positive = opening side, negative = dome side
        projections = vertices @ axis
        
        # Keep vertices on DOME side (negative projection along axis)
        # For bowl (axis up): keep z < 0 (dome below opening)
        # For dome (axis down): keep z > 0 (dome above opening)
        threshold = 0.1 * radius  # Small positive to include some beyond center
        keep_mask = projections <= threshold  # KEEP NEGATIVE SIDE (dome side)
        ```
     4. Filter vertices and remap indices
     5. Filter triangles: keep only triangles where ALL three vertices are kept
     6. Rebuild mesh with filtered vertices and remapped triangle indices
     7. Rotate if needed (from default orientation to actual axis)
     8. **CRITICAL**: Translate to final center position
        ```python
        mesh.translate(center)
        # The center in params is already the sphere center position
        # No additional offset needed!
        ```
   - **Complete example for bowl** (CORRECTED):
     ```python
     mesh = o3d.geometry.TriangleMesh.create_sphere(radius=radius, resolution=20)
     vertices = np.asarray(mesh.vertices)
     triangles = np.asarray(mesh.triangles)
     
     axis = np.array(comp_params.get('axis', [0, 0, 1]))  # Default: bowl (axis up)
     axis = axis / np.linalg.norm(axis)
     
     # Keep vertices on DOME side (OPPOSITE to axis direction)
     # axis points from dome to opening, so dome is at negative projection
     projections = vertices @ axis
     threshold = 0.1 * radius  # Small positive buffer
     keep_mask = projections <= threshold  # Keep negative side = dome side
     
     old_to_new = np.full(len(vertices), -1)
     old_to_new[keep_mask] = np.arange(np.sum(keep_mask))
     new_vertices = vertices[keep_mask]
     
     new_triangles = []
     for tri in triangles:
         if np.all(keep_mask[tri]):
             new_triangles.append([old_to_new[tri[0]], old_to_new[tri[1]], old_to_new[tri[2]]])
     
     mesh = o3d.geometry.TriangleMesh()
     mesh.vertices = o3d.utility.Vector3dVector(new_vertices)
     mesh.triangles = o3d.utility.Vector3iVector(np.array(new_triangles))
     
     # Rotate if axis not aligned with default [0,0,1]
     default_axis = np.array([0, 0, 1])
     if np.abs(np.dot(default_axis, axis) - 1.0) > 1e-6:
         v = np.cross(default_axis, axis)
         s = np.linalg.norm(v)
         c = np.dot(default_axis, axis)
         if s > 1e-6:
             vx = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
             R = np.eye(3) + vx + vx @ vx * ((1 - c) / (s**2))
             mesh.rotate(R, center=[0,0,0])
     
     # Translate to final position - center is already at sphere center!
     mesh.translate(center)
     ```
   
   **TORUS**: create_torus(torus_radius, tube_radius)
   - Local origin: **CENTER at [0, 0, 0]**
   - Already centered! Just translate to final position
   - Transform: `mesh.translate(center)`
   
   - Set colors: semi-transparent (e.g., [0.7, 0.3, 0.3])
   - Return list: [mesh1, mesh2, ...]

REQUIREMENTS:
- Import necessary libraries: numpy as np, open3d as o3d, sklearn if needed, scipy.optimize if using optimization
- **FITTING QUALITY**: Prioritize tight enclosure - minimize gap between primitive and point cloud
- Can use iterative refinement, optimization (scipy.optimize), or analytical methods
- Add small margin (1-2%) to parameters to ensure complete enclosure
- DO NOT add verification/assertion code
- **CRITICAL**: Sphere, Cylinder, Cone, Torus are already centered at origin - just translate!
- **CRITICAL**: Box origin is at corner [0,0,0] - must subtract dims/2 before other transforms!
- Ensure meshes EXACTLY align with point cloud by correct transformations

Return ONLY executable Python code (both functions), no markdown, no explanations.

Example for an oriented box (CORRECT with rotation):
```python
import numpy as np
import open3d as o3d

def extract_parameters(points):
    # Compute bounds first
    min_bounds = np.min(points, axis=0)
    max_bounds = np.max(points, axis=0)
    
    # Use geometric center of bounding box (not mean of points!)
    center = (min_bounds + max_bounds) / 2
    
    # Compute PCA for orientation
    centered = points - center
    cov = np.cov(centered.T)
    eigenvalues, eigenvectors = np.linalg.eig(cov)
    idx = eigenvalues.argsort()[::-1]
    rotation_matrix = eigenvectors[:, idx]
    
    # Ensure right-handed coordinate system
    if np.linalg.det(rotation_matrix) < 0:
        rotation_matrix[:, 2] *= -1
    
    # Check if axis-aligned (all three axes)
    alignment_threshold = 0.95
    x_aligned = max(abs(rotation_matrix[0, 0]), abs(rotation_matrix[1, 0]), abs(rotation_matrix[2, 0])) > alignment_threshold
    y_aligned = max(abs(rotation_matrix[0, 1]), abs(rotation_matrix[1, 1]), abs(rotation_matrix[2, 1])) > alignment_threshold
    z_aligned = max(abs(rotation_matrix[0, 2]), abs(rotation_matrix[1, 2]), abs(rotation_matrix[2, 2])) > alignment_threshold
    is_axis_aligned = x_aligned and y_aligned and z_aligned
    
    if not is_axis_aligned:
        # Tilted - compute dimensions in local frame
        local_points = centered @ rotation_matrix
        min_local = np.min(local_points, axis=0)
        max_local = np.max(local_points, axis=0)
        dimensions = max_local - min_local
        
        return {{
            'parameters': {{
                'component_0': {{
                    'type': 'box',
                    'center': center.tolist(),
                    'dimensions': dimensions.tolist(),
                    'rotation_matrix': rotation_matrix.tolist()
                }}
            }}
        }}
    else:
        # Axis-aligned - use simple AABB
        dimensions = max_bounds - min_bounds
        
        return {{
            'parameters': {{
                'component_0': {{
                    'type': 'box',
                    'center': center.tolist(),
                    'dimensions': dimensions.tolist()
                }}
            }}
        }}

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
```

Example for axis-aligned box (SIMPLE case without rotation):
```python
def extract_parameters(points):
    min_bounds = np.min(points, axis=0)
    max_bounds = np.max(points, axis=0)
    center = (min_bounds + max_bounds) / 2
    dimensions = max_bounds - min_bounds
    
    return {{
        'parameters': {{
            'component_0': {{
                'type': 'box',
                'center': center.tolist(),
                'dimensions': dimensions.tolist()
            }}
        }}
    }}

def create_visualization_geometry(params):
    geometries = []
    
    for comp_name, comp_params in params['parameters'].items():
        if comp_params['type'] == 'box':
            center = np.array(comp_params['center'])
            dims = np.array(comp_params['dimensions'])
            
            mesh = o3d.geometry.TriangleMesh.create_box(
                width=dims[0], height=dims[1], depth=dims[2]
            )
            mesh.translate(center - dims/2)
            mesh.paint_uniform_color([0.7, 0.3, 0.3])
            mesh.compute_vertex_normals()
            geometries.append(mesh)
    
    return geometries
```

Example for an upright cylinder (SIMPLE axis-aligned case):
```python
import numpy as np
import open3d as o3d

def extract_parameters(points):
    # Compute bounds
    min_bounds = np.min(points, axis=0)
    max_bounds = np.max(points, axis=0)
    
    # Initial center estimate from bounding box
    center_xy = (min_bounds[:2] + max_bounds[:2]) / 2
    center_z = (min_bounds[2] + max_bounds[2]) / 2
    
    # For cylinder: optimize XY center to minimize max radius needed
    # (ensures tight fit around points)
    xy_points = points[:, :2]
    
    # Iterative refinement: adjust center to minimize max distance
    for _ in range(10):
        distances = np.linalg.norm(xy_points - center_xy, axis=1)
        # Find the farthest points
        far_idx = np.argsort(distances)[-10:]
        # Adjust center toward the centroid of farthest points
        center_xy = 0.9 * center_xy + 0.1 * np.mean(xy_points[far_idx], axis=0)
    
    # Final radius: max distance from optimized center
    radius = np.max(np.linalg.norm(xy_points - center_xy, axis=1))
    
    # Add small margin to ensure enclosure
    radius *= 1.01
    
    # Height from Z bounds
    height = max_bounds[2] - min_bounds[2]
    center = np.array([center_xy[0], center_xy[1], center_z])
    
    return {{
        'parameters': {{
            'component_0': {{
                'type': 'cylinder',
                'center': center.tolist(),
                'radius': float(radius),
                'height': float(height)
            }}
        }}
    }}

def create_visualization_geometry(params):
    geometries = []
    
    for comp_name, comp_params in params['parameters'].items():
        if comp_params['type'] == 'cylinder':
            center = np.array(comp_params['center'])
            radius = comp_params['radius']
            height = comp_params['height']
            
            # Create Z-aligned cylinder (already centered at origin!)
            mesh = o3d.geometry.TriangleMesh.create_cylinder(
                radius=radius, 
                height=height
            )
            # Cylinder is already centered, just translate to final position
            mesh.translate(center)
            mesh.paint_uniform_color([0.7, 0.3, 0.3])
            mesh.compute_vertex_normals()
            geometries.append(mesh)
    
    return geometries
```
                'radius': float(radius),
                'height': float(height)
            }}
        }}
    }}

def create_visualization_geometry(params):
    geometries = []
    
    for comp_name, comp_params in params['parameters'].items():
        if comp_params['type'] == 'cylinder':
            center = np.array(comp_params['center'])
            radius = comp_params['radius']
            height = comp_params['height']
            
            # Create Z-aligned cylinder (already centered at origin!)
            mesh = o3d.geometry.TriangleMesh.create_cylinder(
                radius=radius, 
                height=height
            )
            # Cylinder is already centered, just translate to final position
            mesh.translate(center)
            mesh.paint_uniform_color([0.7, 0.3, 0.3])
            mesh.compute_vertex_normals()
            geometries.append(mesh)
    
    return geometries
```

Example for an ellipsoid (CORRECT - use PCA for orientation, actual extents for size):
```python
import numpy as np
import open3d as o3d

def extract_parameters(points):
    # Step 1: Compute initial center
    min_bounds = np.min(points, axis=0)
    max_bounds = np.max(points, axis=0)
    center = (min_bounds + max_bounds) / 2
    
    # Step 2: ALWAYS compute PCA to check orientation
    centered = points - center
    cov = np.cov(centered.T)
    eigenvalues, eigenvectors = np.linalg.eig(cov)
    idx = eigenvalues.argsort()[::-1]
    rotation_matrix = eigenvectors[:, idx]
    if np.linalg.det(rotation_matrix) < 0:
        rotation_matrix[:, 2] *= -1
    
    # Step 3: Check alignment with world axes (ALL three axes)
    threshold = 0.95
    x_aligned = max(abs(rotation_matrix[0, 0]), abs(rotation_matrix[1, 0]), abs(rotation_matrix[2, 0])) > threshold
    y_aligned = max(abs(rotation_matrix[0, 1]), abs(rotation_matrix[1, 1]), abs(rotation_matrix[2, 1])) > threshold
    z_aligned = max(abs(rotation_matrix[0, 2]), abs(rotation_matrix[1, 2]), abs(rotation_matrix[2, 2])) > threshold
    is_axis_aligned = x_aligned and y_aligned and z_aligned
    
    # Step 4: Compute radii in local frame
    local_points = centered @ rotation_matrix
    # CRITICAL: Use actual geometric extents, NOT PCA variance!
    radii = np.array([np.max(np.abs(local_points[:, i])) for i in range(3)])
    radii *= 1.01  # Add margin
    
    # Step 5: Return parameters based on alignment
    if not is_axis_aligned:
        return {{
            'parameters': {{
                'component_0': {{
                    'type': 'ellipsoid',
                    'center': center.tolist(),
                    'radii': radii.tolist(),
                    'rotation_matrix': rotation_matrix.tolist()
                }}
            }}
        }}
    else:
        return {{
            'parameters': {{
                'component_0': {{
                    'type': 'ellipsoid',
                    'center': center.tolist(),
                    'radii': radii.tolist()
                }}
            }}
        }}

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
```

Example for a tilted/oriented cylinder:
```python
import numpy as np
import open3d as o3d

def extract_parameters(points):
    # Step 1: Initial center
    min_bounds = np.min(points, axis=0)
    max_bounds = np.max(points, axis=0)
    center = (min_bounds + max_bounds) / 2
    
    # Step 2: ALWAYS compute PCA
    centered = points - center
    cov = np.cov(centered.T)
    eigenvalues, eigenvectors = np.linalg.eig(cov)
    idx = eigenvalues.argsort()[::-1]
    rotation_matrix = eigenvectors[:, idx]
    if np.linalg.det(rotation_matrix) < 0:
        rotation_matrix[:, 2] *= -1
    
    # Step 3: Check if cylinder axis (longest direction) aligns with Z
    # For cylinder: primary axis (longest dimension) is the FIRST principal component
    cylinder_axis = rotation_matrix[:, 0]  # First principal component = height direction
    
    # Check if cylinder axis aligns with world Z axis [0,0,1]
    # Use dot product: if axis ≈ [0,0,±1], then axis·[0,0,1] ≈ ±1
    z_alignment = abs(np.dot(cylinder_axis, np.array([0, 0, 1])))
    z_aligned = z_alignment > 0.95
    
    # Step 4: Compute parameters in local frame
    local_points = centered @ rotation_matrix
    # In local frame: axis 0 is cylinder height, axes 1&2 are radial
    height = np.max(local_points[:, 0]) - np.min(local_points[:, 0])
    radius = np.max(np.sqrt(local_points[:, 1]**2 + local_points[:, 2]**2))
    radius *= 1.01
    height *= 1.01
    
    if not z_aligned:
        # Tilted cylinder - store axis direction
        return {{
            'parameters': {{
                'component_0': {{
                    'type': 'cylinder',
                    'center': center.tolist(),
                    'radius': float(radius),
                    'height': float(height),
                    'axis': cylinder_axis.tolist()
                }}
            }}
        }}
    else:
        # Upright cylinder
        return {{
            'parameters': {{
                'component_0': {{
                    'type': 'cylinder',
                    'center': center.tolist(),
                    'radius': float(radius),
                    'height': float(height)
                }}
            }}
        }}

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
```

Example for a hemisphere (bowl with opening up):
```python
import numpy as np
import open3d as o3d
from sklearn.decomposition import PCA

def extract_parameters(points):
    # Step 1: Compute bounds and initial center estimate
    min_bounds = np.min(points, axis=0)
    max_bounds = np.max(points, axis=0)
    center = (min_bounds + max_bounds) / 2

    # Step 2: Perform PCA to determine orientation
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

    # Step 4: Compute radius and determine opening direction
    local_points = centered @ rotation_matrix
    radius = np.max(np.sqrt(local_points[:, 0]**2 + local_points[:, 1]**2 + local_points[:, 2]**2))
    
    # ROBUST direction determination: analyze shape, not just point count
    # Key: Opening side has LARGER XY spread (wider), dome side has SMALLER spread (curved)
    top_points = points[centered[:, 2] > 0]
    bottom_points = points[centered[:, 2] < 0]
    
    if len(top_points) > 10 and len(bottom_points) > 10:
        top_center_xy = np.mean(top_points[:, :2], axis=0)
        bottom_center_xy = np.mean(bottom_points[:, :2], axis=0)
        
        top_radius = np.max(np.linalg.norm(top_points[:, :2] - top_center_xy, axis=1))
        bottom_radius = np.max(np.linalg.norm(bottom_points[:, :2] - bottom_center_xy, axis=1))
        
        # If top has larger radius → opening at top (bowl)
        opening_at_top = top_radius > bottom_radius * 1.1  # 10% threshold
    else:
        # Fallback: use point count
        opening_at_top = len(top_points) > len(bottom_points)

    # CRITICAL: For hemisphere, center should be at the opening plane (rim level)
    # axis points FROM dome TO opening
    if opening_at_top:
        # Bowl: opening at top, dome at bottom
        # axis points UP from dome to opening
        axis_dir = np.array([0, 0, 1]) if is_axis_aligned else rotation_matrix[:, 2]
        # Adjust center to rim level (highest points = opening level)
        center[2] = max_bounds[2] if is_axis_aligned else center[2] + radius * 0.3
    else:
        # Dome/inverted: opening at bottom, dome at top
        # axis points DOWN from dome to opening
        axis_dir = np.array([0, 0, -1]) if is_axis_aligned else -rotation_matrix[:, 2]
        # Adjust center to base level (lowest points = opening level)
        center[2] = min_bounds[2] if is_axis_aligned else center[2] - radius * 0.3

    # Add margin
    radius *= 1.02

    if not is_axis_aligned:
        return {{
            'parameters': {{
                'component_0': {{
                    'type': 'hemisphere',
                    'center': center.tolist(),
                    'radius': float(radius),
                    'axis': axis_dir.tolist()
                }}
            }}
        }}
    else:
        return {{
            'parameters': {{
                'component_0': {{
                    'type': 'hemisphere',
                    'center': center.tolist(),
                    'radius': float(radius),
                    'axis': axis_dir.tolist()
                }}
            }}
        }}

def create_visualization_geometry(params):
    geometries = []
    for comp_name, comp_params in params['parameters'].items():
        if comp_params['type'] == 'hemisphere':
            center = np.array(comp_params['center'])
            radius = comp_params['radius']
            
            # Create full sphere
            mesh = o3d.geometry.TriangleMesh.create_sphere(radius=radius, resolution=20)
            vertices = np.asarray(mesh.vertices)
            triangles = np.asarray(mesh.triangles)
            
            # Get axis direction (from dome to opening)
            axis = np.array(comp_params.get('axis', [0, 0, 1]))  # Default: bowl (up)
            axis = axis / np.linalg.norm(axis)
            
            # Keep vertices on DOME side (OPPOSITE to axis direction)
            # axis points from dome to opening, so keep negative projection (dome side)
            projections = vertices @ axis
            threshold = 0.1 * radius  # Small positive buffer
            keep_mask = projections <= threshold  # Keep dome side (negative/zero projection)
            
            # Remap vertex indices
            old_to_new = np.full(len(vertices), -1)
            old_to_new[keep_mask] = np.arange(np.sum(keep_mask))
            new_vertices = vertices[keep_mask]
            
            # Filter triangles
            new_triangles = []
            for tri in triangles:
                if np.all(keep_mask[tri]):
                    new_triangles.append([old_to_new[tri[0]], old_to_new[tri[1]], old_to_new[tri[2]]])
            
            if len(new_triangles) > 0:
                mesh = o3d.geometry.TriangleMesh()
                mesh.vertices = o3d.utility.Vector3dVector(new_vertices)
                mesh.triangles = o3d.utility.Vector3iVector(np.array(new_triangles))
                
                # Rotate if needed
                default_axis = np.array([0, 0, 1])
                if np.abs(np.dot(default_axis, axis) - 1.0) > 1e-6:
                    v = np.cross(default_axis, axis)
                    s = np.linalg.norm(v)
                    c = np.dot(default_axis, axis)
                    if s > 1e-6:
                        vx = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
                        R = np.eye(3) + vx + vx @ vx * ((1 - c) / (s**2))
                        mesh.rotate(R, center=[0,0,0])
                
                # Translate to final position
                mesh.translate(center)
                mesh.paint_uniform_color([0.9, 0.9, 0.85])
                mesh.compute_vertex_normals()
                geometries.append(mesh)
    
    return geometries
```

REMEMBER:
- ALWAYS run PCA for ALL primitives to check orientation
- Check alignment on ALL three axes (x, y, z) with threshold=0.95
- For ellipsoid: use actual extents `np.max(np.abs(local_points[:, i]))`, NOT variance!
- For cylinder: check if primary axis aligns with Z
- For hemisphere: axis points from base to dome, center is at the opening plane (rim/base level)
- Store rotation_matrix/axis ONLY if not axis-aligned
- Add 1-2% margin to dimensions/radii
- Open3D Box: corner at [0,0,0], translate by (center - dims/2)
- Open3D Cylinder/Sphere/Ellipsoid/Hemisphere: centered at origin, just translate(center)

Example for optimized fitting with scipy (ADVANCED):
```python
import numpy as np
import open3d as o3d
from scipy.optimize import minimize

def extract_parameters(points):
    min_bounds = np.min(points, axis=0)
    max_bounds = np.max(points, axis=0)
    
    # Initial guess: bounding box center
    initial_center = (min_bounds + max_bounds) / 2
    
    # For sphere: optimize center to minimize max distance
    def objective(center):
        distances = np.linalg.norm(points - center, axis=1)
        return np.max(distances)  # Minimize the max distance (radius)
    
    # Optimize
    result = minimize(objective, initial_center, method='Nelder-Mead')
    optimal_center = result.x
    
    # Compute optimal radius
    distances = np.linalg.norm(points - optimal_center, axis=1)
    radius = np.max(distances) * 1.01  # Small margin
    
    return {{
        'parameters': {{
            'component_0': {{
                'type': 'sphere',
                'center': optimal_center.tolist(),
                'radius': float(radius)
            }}
        }}
    }}

def create_visualization_geometry(params):
    geometries = []
    
    for comp_name, comp_params in params['parameters'].items():
        if comp_params['type'] == 'sphere':
            center = np.array(comp_params['center'])
            radius = comp_params['radius']
            
            mesh = o3d.geometry.TriangleMesh.create_sphere(radius=radius)
            mesh.translate(center)
            mesh.paint_uniform_color([0.7, 0.3, 0.3])
            mesh.compute_vertex_normals()
            geometries.append(mesh)
    
    return geometries
```

Now generate code for {object_name}."""
        
        return task_desc + cloud_info + primitives_info + requirements
    
    def parameterize(self, point_cloud_paths, object_name, subtask_semantic=None):
        """Main geometric parameterization workflow
        
        Args:
            point_cloud_paths: Single path string or list of paths
            object_name: Object name
            subtask_semantic: Task description
        """
        # Normalize input: convert to list
        if isinstance(point_cloud_paths, str):
            point_cloud_paths = [point_cloud_paths]
        
        print(f"\n{'='*60}\nGeometric Parameterization: {object_name}\n{'='*60}")
        
        # Step 1: Loading point clouds
        print(f"[Step 1] Loading point cloud(s)...")
        points_dict = {}
        total_points = 0
        
        for path in point_cloud_paths:
            pcd = o3d.io.read_point_cloud(path)
            points = np.asarray(pcd.points)
            part_name = os.path.splitext(os.path.basename(path))[0].replace(object_name + '_', '').replace('_', ' ')
            if len(point_cloud_paths) == 1:
                part_name = 'component'
            points_dict[part_name] = points
            total_points += len(points)
            print(f"  ✓ {part_name}: {len(points)} points")
        
        # Step 2: LLM auto-select primitives for each part
        print(f"[Step 2] LLM auto-selecting primitives...")
        primitives_dict = {}
        for part_name, points in points_dict.items():
            part_semantic = f"{object_name} - {part_name}" if len(points_dict) > 1 else object_name
            selected = self.select_primitives_with_llm(part_semantic, points, subtask_semantic)
            primitives_dict[part_name] = selected
        
        # Step 3: Assembling prompt
        print(f"[Step 3] Assembling prompt...")
        prompt = self.assemble_prompt(object_name, points_dict, primitives_dict, subtask_semantic)
        print(f"  ✓ Done")
        
        # Step 4: LLM generating algorithm and visualization functions
        print(f"[Step 4] LLM generating algorithm and visualization...")
        response = self.llm.generate_code(prompt, self.system_prompt, temperature=0.3)
        algorithm_code = self.llm.extract_code_block(response)
        print("-" * 60)
        print(algorithm_code)
        print("-" * 60)
        
        # Step 5: Executing algorithm to extract parameters
        print(f"[Step 5] Executing algorithm...")
        params = {}
        visualization_func = None
        
        try:
            namespace = {'np': np, 'numpy': np, 'o3d': o3d}
            
            try:
                from scipy import optimize, spatial
                import scipy
                namespace.update({'scipy': scipy, 'optimize': optimize, 'spatial': spatial})
            except ImportError:
                print("  ⚠ scipy not installed")
            
            try:
                from sklearn.cluster import DBSCAN, KMeans
                import sklearn, sklearn.cluster
                namespace.update({'sklearn': sklearn, 'DBSCAN': DBSCAN, 'KMeans': KMeans})
                import sys
                sys.modules.setdefault('sklearn', sklearn)
                sys.modules.setdefault('sklearn.cluster', sklearn.cluster)
            except ImportError as e:
                print(f"  ⚠ sklearn not installed: {e}")
            
            exec(algorithm_code, namespace)
            
            # Extract parameters
            if 'extract_parameters' in namespace:
                input_data = list(points_dict.values())[0] if len(points_dict) == 1 else points_dict
                params = namespace['extract_parameters'](input_data)
                print(f"  ✓ Successfully extracted parameters:")
                for key, value in params.items():
                    if isinstance(value, (list, tuple)) and len(value) <= 3:
                        print(f"      {key}: {value}")
                    else:
                        print(f"      {key}: {type(value).__name__}")
            else:
                print(f"  ✗ No extract_parameters function found")
            
            if 'create_visualization_geometry' in namespace:
                visualization_func = namespace['create_visualization_geometry']
                print(f"  ✓ Successfully extracted visualization function")
            else:
                print(f"  ⚠ No create_visualization_geometry function found")
                
        except Exception as e:
            print(f"  ✗ Error: {e}")
            import traceback
            traceback.print_exc()
        
        return {
            'object_name': object_name,
            'subtask': subtask_semantic,
            'selected_primitives': primitives_dict,
            'algorithm_code': algorithm_code,
            'parameters': params,
            'visualization_func': visualization_func,
            'point_cloud_paths': point_cloud_paths,
            'num_points': total_points
        }
    
    def save_results(self, result, output_dir="geometric_parameters"):
        """Save separately: parameters JSON + algorithm PY"""
        os.makedirs(output_dir, exist_ok=True)
        name = result['object_name'].replace(' ', '_')
        
        params = result['parameters']
        actual_params = params.get('parameters', params) if isinstance(params, dict) else params
        
        # 1. Save parameters JSON
        params_file = os.path.join(output_dir, f"{name}_params.json")
        params_data = {
            'object_name': result['object_name'],
            'subtask': result.get('subtask'),
            'selected_primitives': result['selected_primitives'],
            'point_cloud_paths': result['point_cloud_paths'],
            'num_points': result['num_points'],
            'parameters': actual_params
        }
        with open(params_file, 'w', encoding='utf-8') as f:
            json.dump(params_data, f, indent=2, ensure_ascii=False)
        print(f"\n✓ Parameters: {params_file}")
        
        # 2. Save algorithm PY
        algorithm_file = os.path.join(output_dir, f"{name}_algorithm.py")
        with open(algorithm_file, 'w', encoding='utf-8') as f:
            f.write(f"# {result['object_name']} - Geometric Analysis Algorithm\n")
            f.write(f"# Primitives: {result['selected_primitives']}\n\n")
            f.write(result['algorithm_code'])
        print(f"✓ Algorithm: {algorithm_file}")


def test_geometric_parameterization():
    """Test"""
    try:
        llm = LLMInterface()
        parameterizer = GeometricParameterizer(llm)

        test_objects = {
            # "teacup": ["teacup.ply"],
            # "tomato": ["tomato.ply"],
            # "cookie_box": ["cookie_box.ply"],
            # "laptop": ["laptop_base.ply", "laptop_screen.ply"],
            "bowl": ["bowl.ply"],
        }
        results = {}
        
        for obj_name, obj_files in test_objects.items():
            paths = [os.path.join("point_cloud", f) for f in obj_files]
            
            # Check if files exist
            missing = [p for p in paths if not os.path.exists(p)]
            if missing:
                print(f"\nWarning: Missing files for {obj_name}: {missing}")
                continue
            
            result = parameterizer.parameterize(paths, obj_name)
            results[obj_name] = result
            parameterizer.save_results(result)
            print(f"\n{'-'*60}\n")
        
        print(f"\n{'='*60}\nCompleted: {len(results)} objects\n{'='*60}")
        return results
    except Exception as e:
        print(f"\nFailed: {e}")
        import traceback
        traceback.print_exc()


if __name__ == "__main__":
    test_geometric_parameterization()
