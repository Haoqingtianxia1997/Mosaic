"""
Trajectory Generation Module - Generate robot manipulation trajectories using LLM
Based on EmbodiedCoder paper methodology
"""
import numpy as np
import os
import json
from llm_interface import LLMInterface


class TrajectoryGenerator:
    """Trajectory Generator - Generate robot manipulation trajectories based on geometric parameters"""
    
    def __init__(self, llm_interface):
        """
        Initialize trajectory generator
        
        Args:
            llm_interface: LLM interface instance
        """
        self.llm = llm_interface
        self.system_prompt = self._create_system_prompt()
    
    def _create_system_prompt(self):
        """Create system prompt - Based on trajectory generation method in paper"""
        return """You are an expert in robotic manipulation and trajectory planning. Your task is to generate executable Python code for robot manipulation trajectories.

Given geometric parameters of an object, you should:
1. Generate waypoints for approach, grasp, manipulation, and retreat phases
2. Consider collision avoidance and natural motion
3. Define grasp poses based on object geometry
4. Create smooth trajectories with proper timing
5. CRITICAL: End position must be at SAME HEIGHT as start but DIFFERENT XY location (avoid returning to origin)
6. CRITICAL: During manipulation, ensure gripper/end-effector CONTACTS the object surface (use geometric parameters to calculate contact points)

Return ONLY executable Python code that:
- Defines waypoints as lists of [x, y, z, roll, pitch, yaw] or transformation matrices
- Includes functions to generate trajectory points
- Has clear comments explaining each phase
- Uses numpy for calculations
- Ensures START and END are at same Z height but different XY positions (minimum 15cm XY distance)
- Calculates contact points on object surface using geometric parameters (radius, dimensions, etc.)

TRAJECTORY PHASES (follow this structure):
1. START: Safe approach position (typically 15-20cm above/away from object, at a specific height)
2. PRE-GRASP: Position close to grasp point (5-10cm away)
3. GRASP: AT OBJECT SURFACE - Calculate exact contact point using geometry (e.g., cylinder surface at radius distance)
4. CLOSE_GRIPPER: Same position, gripper closed
5. LIFT/MANIPULATE: Execute task (move up, rotate, pour, etc.)
6. PLACE/LOWER: Lower back to original height if needed
7. END: At SAME HEIGHT as start, but DIFFERENT XY position (move 15-30cm away in X or Y direction)

HUMAN-CENTRIC MANIPULATION ANALYSIS:
Before generating trajectories, analyze WHERE humans naturally grasp objects:
- **Bowl/Cup (Hemisphere/Cylinder)**: Humans grasp at the RIM/EDGE from the side (2-handed) or at handle (1-handed)
  * For bowls: Fingers wrap around rim opening, NOT at the bottom or dome center
  * Grasp point is at the OPENING EDGE (where you can see inside), offset horizontally from center
- **Box/Package**: Grasp at EDGES or CORNERS for leverage, not flat surfaces
  * Fingers naturally grip edges where two surfaces meet
- **Sphere/Ball**: Grasp at EQUATOR (midsection) where hand wraps around naturally
- **Cylinder (vertical)**: Grasp at MID-HEIGHT for balance, like holding a bottle
- **Key principle**: Contact points must match how humans physically interact with objects in daily life

CONTACT POINT CALCULATION RULES:
- **Hemisphere (Bowl)**: 
  * Understand geometry: 'center' is at OPENING PLANE (not dome center), 'axis' points from dome to opening
  * Grasp at RIM: contact = center + horizontal_direction * radius * 0.9
  * horizontal_direction is perpendicular to axis (e.g., if axis=[0,0,1], use direction=[1,0,0] or [0,1,0])
  * CRITICAL: Rim is where humans hold bowls - at the opening edge, NOT at bottom of dome!
- **Cylinder**: Contact at center ± [radius * cos(angle), radius * sin(angle), z_position]
- **Box WITHOUT rotation_matrix**: Contact at center ± [width/2, depth/2, height/2] on face/edge
- **Box WITH rotation_matrix**: 
  1. Get local contact offset (e.g., [0, 0, height/2] for top surface)
  2. Transform to world: contact = center + rotation_matrix @ local_offset
  3. CRITICAL: MUST use rotation matrix to find actual surface position!
- **Sphere/Ellipsoid**: Contact at center ± radius * direction_vector
- **Always add surface contact**: Don't hover in air - touch the object!
- **VERIFY Z coordinates**: All waypoint Z values MUST be positive (above ground)!

ROTATION MATRIX HANDLING (CRITICAL):
- If a component has 'rotation_matrix' field, the object is TILTED/ROTATED
- To find surface contact points on rotated objects:
  ```python
  R = np.array(component['rotation_matrix'])  # 3x3 rotation matrix
  center = np.array(component['center'])
  
  # For box top surface: local offset [0, 0, height/2]
  local_offset = np.array([0, 0, component['dimensions'][2]/2])
  world_contact = center + R @ local_offset  # Transform to world frame
  
  # For box side surface: local offset [width/2, 0, 0]
  local_offset = np.array([component['dimensions'][0]/2, 0, 0])
  world_contact = center + R @ local_offset
  ```
- NEVER ignore rotation_matrix! It changes where the surface actually is!

HEIGHT CONSISTENCY RULES:
- If start Z = h, then end Z = h (same height, different XY)
- During manipulation, Z can change (lift, lower, etc.)
- But final waypoint should return to original approach height
- This creates natural "pick from A, place at B" motion

Example output format:
```python
import numpy as np

# Manipulation trajectory for coffee cup - pick and relocate
def generate_cup_manipulation_trajectory(params):
    waypoints = []
    
    # Extract geometry - handle nested structure
    if 'component_0' in params:
        cylinder = params['component_0']
        radius = cylinder['radius']
        height = cylinder['height']
        center = np.array(cylinder['center'])
    else:
        # Fallback for flat structure
        radius = params.get('radius', 0.03)
        height = params.get('height', 0.08)
        center = np.array(params.get('center', [0, 0, height/2]))
    
    # Define approach height (e.g., mid-height of cup)
    approach_z = center[2]
    
    # Phase 1: START - Approach from side at specific height
    start_pos = [center[0] + radius + 0.20, center[1], approach_z]
    waypoints.append({
        'position': start_pos,
        'orientation': [0, 0, np.pi/2],  # Facing the cup
        'gripper_state': 'open',
        'description': 'Start: Safe approach position from side at height {:.3f}m'.format(approach_z)
    })
    
    # Phase 2: Move closer to handle (pre-grasp)
    handle_angle = 0  # Handle at 0 degrees (positive X direction)
    handle_contact = center + np.array([radius * np.cos(handle_angle), radius * np.sin(handle_angle), 0])
    
    waypoints.append({
        'position': [handle_contact[0] + 0.05, handle_contact[1], approach_z],
        'orientation': [0, 0, np.pi/2],
        'gripper_state': 'open',
        'description': 'Pre-grasp: Approaching handle'
    })
    
    # Phase 3: GRASP - AT SURFACE (contact point on cylinder)
    waypoints.append({
        'position': handle_contact.tolist(),  # Exact contact with cylinder surface
        'orientation': [0, 0, np.pi/2],
        'gripper_state': 'open',
        'description': 'Grasp: AT handle surface (radius={:.3f}m from center)'.format(radius)
    })
    
    # Phase 4: Close gripper
    waypoints.append({
        'position': handle_contact.tolist(),
        'orientation': [0, 0, np.pi/2],
        'gripper_state': 'close',
        'description': 'Close gripper on handle'
    })
    
    # Phase 5: Lift cup up 15cm
    waypoints.append({
        'position': [handle_contact[0], handle_contact[1], center[2] + 0.15],
        'orientation': [0, 0, np.pi/2],
        'gripper_state': 'close',
        'description': 'Lift cup upward'
    })
    
    # Phase 6: Move to side while maintaining lifted height
    waypoints.append({
        'position': [handle_contact[0] - 0.20, handle_contact[1] + 0.10, center[2] + 0.15],
        'orientation': [0, 0, np.pi/2],
        'gripper_state': 'close',
        'description': 'Move to new location while lifted'
    })
    
    # Phase 7: Lower back to original approach height
    waypoints.append({
        'position': [handle_contact[0] - 0.20, handle_contact[1] + 0.10, approach_z],
        'orientation': [0, 0, np.pi/2],
        'gripper_state': 'close',
        'description': 'Lower to original height'
    })
    
    # Phase 8: END - At SAME HEIGHT as start, but DIFFERENT XY location
    end_pos = [center[0] - radius - 0.25, center[1] + 0.15, approach_z]  # Same Z as start!
    waypoints.append({
        'position': end_pos,
        'orientation': [0, 0, np.pi/2],
        'gripper_state': 'open',
        'description': 'End: Same height as start ({:.3f}m), but moved to new XY location'.format(approach_z)
    })
    
    return waypoints
```

Example for ROTATED BOX (e.g., tilted laptop screen):
```python
import numpy as np

def generate_laptop_screen_push_trajectory(params):
    waypoints = []
    
    # Extract screen geometry (rotated box)
    screen = params.get('screen_component_0', params.get('component_0'))
    center = np.array(screen['center'])
    dims = np.array(screen['dimensions'])  # [width, depth, height]
    
    # Check if object is rotated
    if 'rotation_matrix' in screen:
        R = np.array(screen['rotation_matrix'])  # 3x3 rotation matrix
        is_rotated = True
    else:
        R = np.eye(3)
        is_rotated = False
    
    # Define approach height (use center Z + some offset)
    approach_z = center[2] + 0.10  # 10cm above center
    
    # Phase 1: START - Safe position above screen
    start_pos = [center[0], center[1], approach_z]
    waypoints.append({
        'position': start_pos,
        'orientation': [0, 0, 0],
        'gripper_state': 'open',
        'description': 'Start: Safe approach position at height {:.3f}m'.format(approach_z)
    })
    
    # Phase 2: Calculate contact point on TOP surface of (rotated) screen
    if is_rotated:
        # For rotated box: find top surface in world frame
        # Local offset for top surface: [0, 0, height/2]
        local_top_offset = np.array([0, 0, dims[2]/2])
        world_top_contact = center + R @ local_top_offset
    else:
        # For axis-aligned box: simple offset
        world_top_contact = center + np.array([0, 0, dims[2]/2])
    
    # Phase 3: Move closer to contact point
    pre_grasp_pos = world_top_contact + np.array([0, 0, 0.05])  # 5cm above contact
    waypoints.append({
        'position': pre_grasp_pos.tolist(),
        'orientation': [0, 0, 0],
        'gripper_state': 'open',
        'description': 'Pre-grasp: Approaching screen surface'
    })
    
    # Phase 4: GRASP - At surface contact (MUST touch surface!)
    waypoints.append({
        'position': world_top_contact.tolist(),
        'orientation': [0, 0, 0],
        'gripper_state': 'open',
        'description': 'Grasp: AT screen top surface (contact point)'
    })
    
    # Phase 5: Push/manipulate
    if is_rotated:
        # Push along surface normal direction (downward in local frame)
        local_push_offset = np.array([0, 0, -0.05])  # Push 5cm inward
        push_pos = center + R @ (local_top_offset + local_push_offset)
    else:
        push_pos = world_top_contact - np.array([0, 0, 0.05])
    
    waypoints.append({
        'position': push_pos.tolist(),
        'orientation': [0, 0, 0],
        'gripper_state': 'close',
        'description': 'Manipulation: Push screen'
    })
    
    # Phase 6: Return to pre-grasp height
    waypoints.append({
        'position': pre_grasp_pos.tolist(),
        'orientation': [0, 0, 0],
        'gripper_state': 'open',
        'description': 'Return to safe height'
    })
    
    # Phase 7: END - Same height as start, different XY
    end_pos = [center[0] + 0.20, center[1] + 0.15, approach_z]  # Same Z!
    waypoints.append({
        'position': end_pos,
        'orientation': [0, 0, 0],
        'gripper_state': 'open',
        'description': 'End: Same height as start ({:.3f}m), different XY location'.format(approach_z)
    })
    
    return waypoints
```

Focus on practical, collision-free trajectories suitable for real robot execution.
CRITICAL REMINDERS:
1. START Z = END Z (same height, different location)
2. Contact points must touch object surface (use radius, width/2, etc.)
3. For rotated objects: Use rotation_matrix @ local_offset to find world contact points!
4. Verify ALL Z coordinates are positive (above ground)!
5. Minimum 15cm XY distance between start and end positions"""
    
    def create_trajectory_prompt(self, object_name, geometric_params, task_description):
        """
        Create trajectory generation prompt
        
        Args:
            object_name: Object name
            geometric_params: Geometric parameters dictionary
            task_description: Task description
            
        Returns:
            Prompt string
        """
        # Format geometric parameters as JSON string, maintaining nested structure
        import json
        params_str = json.dumps(geometric_params, indent=2)
        
        prompt = f"""Generate a robotic manipulation trajectory for the following task:

Object: {object_name}
Task: {task_description}

Geometric Parameters (as nested dictionary):
```python
params = {params_str}
```

IMPORTANT: The params dict has nested structure! For example:
- If params = {{'component_0': {{'type': 'cylinder', 'radius': 0.03, ...}}}}, access via params['component_0']['radius']
- If params = {{'ellipsoid': {{'radius_x': 0.04, ...}}}}, access via params['ellipsoid']['radius_x']
- If params = {{'box': {{'width': 0.15, ...}}}}, access via params['box']['width']

When writing the trajectory function, make sure to access nested parameters correctly!

Generate Python code that defines a complete manipulation trajectory including:
1. START: Approach phase - Move to safe pre-grasp position (15-20cm away from object) at a specific height Z
2. PRE-GRASP: Move closer to grasp location (5-10cm away)
3. GRASP: Position gripper AT OBJECT SURFACE - Calculate exact contact point (e.g., center + radius for cylinder)
4. CLOSE: Close gripper around object at same position
5. MANIPULATION: Execute the main task (pick up, move, rotate, etc.)
6. PLACE/LOWER: If lifted, lower back to original height
7. END: Move to final position - MUST be at SAME HEIGHT Z as start, but DIFFERENT XY location!

CRITICAL REQUIREMENTS:
- **START Z = END Z**: Start and end positions MUST be at the same height (e.g., both at 0.04m)
- **START XY ≠ END XY**: XY positions must be at least 15cm apart (different locations!)
- **SURFACE CONTACT**: Grasp point must be ON the object surface, not floating in air
  * For cylinder: contact_pos = center + [radius * cos(θ), radius * sin(θ), 0]
  * For box: contact_pos = center ± [width/2, depth/2, height/2]
  * For sphere/ellipsoid: contact_pos = center + radius * unit_direction
- Trajectory should follow natural human-like manipulation patterns
- All positions in meters, orientations in radians (RPY: roll, pitch, yaw)
- Include at least 7-10 waypoints for smooth motion
- Consider object geometry for grasp planning (use the provided nested parameters!)
- Add safety margins (e.g., approach from 15cm away, then move closer)
- Define gripper states clearly: 'open', 'close', or float (width in meters)
- For tasks like "pick and move": end should be at same height but different XY location
- For tasks like "pour": include tilting motion, then return to same height as start
- For tasks like "open laptop": push screen from contact point, end at same height as start

Each waypoint MUST have:
- 'position': [x, y, z] in meters
- 'orientation': [roll, pitch, yaw] in radians
- 'gripper_state': 'open' | 'close' | float (width in meters)
- 'description': string explaining this waypoint's purpose

VERIFICATION CHECKLIST:
✓ Start waypoint Z value = End waypoint Z value (same height!)
✓ Start XY and End XY are different (at least 15cm apart)
✓ Grasp waypoint position is ON object surface (calculated using geometry)
✓ All intermediate waypoints use valid geometric calculations

Return ONLY the Python code. Ensure the trajectory is realistic and physically plausible!"""
        
        return prompt
    
    def generate_trajectory(self, object_name, geometric_params, task_description):
        """
        Generate robot manipulation trajectory
        
        Args:
            object_name: Object name
            geometric_params: Geometric parameters dictionary
            task_description: Task description
            
        Returns:
            Dictionary containing trajectory code and execution results
        """
        print(f"\n=== Starting trajectory generation: {object_name} ===")
        print(f"Task: {task_description}")
        
        # Create prompt
        prompt = self.create_trajectory_prompt(object_name, geometric_params, task_description)
        
        # Call LLM to generate trajectory code
        print("Calling LLM to generate trajectory code...")
        response = self.llm.generate_code(prompt, self.system_prompt, temperature=0.4)
        
        # Extract code
        code = self.llm.extract_code_block(response)
        
        print("\nGenerated trajectory code:")
        print("-" * 50)
        print(code)
        print("-" * 50)
        
        # Execute code to generate trajectory
        trajectory = None
        trajectory_function = None
        
        try:
            # Create execution environment
            exec_globals = {
                'np': np,
                'numpy': np,
                'params': geometric_params
            }
            exec_locals = {}
            
            # Execute code
            exec(code, exec_globals, exec_locals)
            
            # Find trajectory generation function
            for name, obj in exec_locals.items():
                if callable(obj) and 'trajectory' in name.lower():
                    trajectory_function = obj
                    print(f"\nFound trajectory generation function: {name}")
                    
                    # Execute function to generate trajectory
                    try:
                        trajectory = obj(geometric_params)
                        print(f"Successfully generated trajectory with {len(trajectory)} waypoints")
                    except Exception as e:
                        print(f"Error executing trajectory function: {e}")
                    break
            
            if trajectory_function is None:
                print("Warning: Trajectory generation function not found")
                
        except Exception as e:
            print(f"Warning: Error executing generated code: {e}")
            import traceback
            traceback.print_exc()
        
        return {
            'object_name': object_name,
            'task': task_description,
            'code': code,
            'trajectory': trajectory,
            'trajectory_function': trajectory_function
        }
    
    def save_trajectory(self, result, output_path):
        """
        Save trajectory code to file
        
        Args:
            result: Return result from generate_trajectory method
            output_path: Output file path
        """
        with open(output_path, 'w', encoding='utf-8') as f:
            f.write(f"# Trajectory for {result['object_name']}\n")
            f.write(f"# Task: {result['task']}\n")
            f.write(f"# Generated by TrajectoryGenerator\n\n")
            f.write("import numpy as np\n\n")
            f.write(result['code'])
        
        print(f"\nTrajectory code saved to: {output_path}")
    
    def save_trajectory_waypoints(self, result, output_path):
        """
        Save trajectory waypoints to JSON file
        
        Args:
            result: Return result from generate_trajectory method
            output_path: Output file path
        """
        if result['trajectory'] is None:
            print("Warning: No trajectory data to save")
            return
        
        # Convert numpy arrays to lists for JSON serialization
        def convert_to_serializable(obj):
            if isinstance(obj, np.ndarray):
                return obj.tolist()
            elif isinstance(obj, dict):
                return {k: convert_to_serializable(v) for k, v in obj.items()}
            elif isinstance(obj, list):
                return [convert_to_serializable(item) for item in obj]
            else:
                return obj
        
        trajectory_data = {
            'object_name': result['object_name'],
            'task': result['task'],
            'num_waypoints': len(result['trajectory']),
            'waypoints': convert_to_serializable(result['trajectory'])
        }
        
        with open(output_path, 'w', encoding='utf-8') as f:
            json.dump(trajectory_data, f, indent=2, ensure_ascii=False)
        
        print(f"Trajectory waypoints saved to: {output_path}")
    

def test_trajectory_generation():
    """
    Test trajectory generation for all objects with saved geometric parameters
    
    Automatically discovers all parameter files and generates trajectories for them.
    """
    print("=" * 60)
    print("Testing Trajectory Generation Module")
    print("=" * 60)
    
    # Initialize LLM and generator
    print("\n[Step 1] Initializing LLM interface and trajectory generator...")
    llm = LLMInterface()
    generator = TrajectoryGenerator(llm)
    
    # Define directories
    params_dir = "geometric_parameters"
    trajectory_dir = "trajectories"
    os.makedirs(trajectory_dir, exist_ok=True)
    
    # Define tasks for each object
    tasks = {
        'teacup': 'Pick up the cup by the handle and lift it 20cm upward, then move it to a new location',
        'tomato': 'Grasp the tomato from the top and lift it gently upward, then place it at a different location',
        'cookie_box': 'Grasp the box from the sides and slide it forward 30cm',
        'laptop': 'Push the laptop screen to fold the screen over the keyboard',
        'bowl': 'Grasp the bowl at the RIM EDGE from the side (where humans naturally hold bowls - at the opening edge, NOT at the dome bottom). The rim contact point is at center + horizontal_offset * radius, where center is at the opening plane. Lift 15cm upward and move to new location.'
    }
    
    # Discover all parameter files
    if not os.path.exists(params_dir):
        print(f"\n✗ Error: Parameters directory not found: {params_dir}")
        return
    
    param_files = [f for f in os.listdir(params_dir) if f.endswith('_params.json')]
    
    if not param_files:
        print(f"\n✗ No parameter files found in {params_dir}")
        return
    
    print(f"\n[Step 2] Found {len(param_files)} objects to process:")
    for pf in param_files:
        print(f"  - {pf}")
    
    # Process each object
    results = {}
    
    for param_file in param_files:
        object_name = param_file.replace('_params.json', '')
        
        print("\n" + "=" * 60)
        print(f"Processing: {object_name}")
        print("=" * 60)
        
        # Load geometric parameters
        params_path = os.path.join(params_dir, param_file)
        with open(params_path, 'r', encoding='utf-8') as f:
            data = json.load(f)
            geometric_params = data.get('parameters', {})
        
        print(f"\n✓ Loaded geometric parameters with {len(geometric_params)} components")
        
        # Get task description
        task_description = tasks.get(object_name, f'Manipulate the {object_name}')
        print(f"✓ Task: {task_description}")
        
        # Generate trajectory
        print(f"\n[Step 3] Generating trajectory...")
        try:
            result = generator.generate_trajectory(
                object_name=object_name,
                geometric_params=geometric_params,
                task_description=task_description
            )
            
            # Save trajectory code
            code_output_path = os.path.join(trajectory_dir, f"{object_name}_trajectory.py")
            generator.save_trajectory(result, code_output_path)
            
            # Save trajectory waypoints
            waypoints_output_path = os.path.join(trajectory_dir, f"{object_name}_waypoints.json")
            generator.save_trajectory_waypoints(result, waypoints_output_path)
            
            results[object_name] = result
            
            print(f"\n✓ Successfully generated trajectory for {object_name}")
            if result['trajectory']:
                print(f"  - Waypoints: {len(result['trajectory'])}")
                print(f"  - Start: {result['trajectory'][0]['position']}")
                print(f"  - End: {result['trajectory'][-1]['position']}")
            
        except Exception as e:
            print(f"\n✗ Error generating trajectory for {object_name}: {e}")
            import traceback
            traceback.print_exc()
    
    # Summary
    print("\n" + "=" * 60)
    print("Summary")
    print("=" * 60)
    print(f"Total objects processed: {len(param_files)}")
    print(f"Successful: {len(results)}")
    print(f"Failed: {len(param_files) - len(results)}")
    
    if results:
        print("\nGenerated trajectories:")
        for obj_name, result in results.items():
            if result['trajectory']:
                print(f"  ✓ {obj_name}: {len(result['trajectory'])} waypoints")
            else:
                print(f"  ✗ {obj_name}: No trajectory generated")
    
    print("\n" + "=" * 60)
    print("Trajectory generation test complete!")
    print("=" * 60)
    
    return results


if __name__ == "__main__":
    # Run test
    test_trajectory_generation()
