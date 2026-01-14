"""
Main Program - Integrating geometric parameterization and trajectory generation
Using point cloud files for complete workflow validation
"""
import os
import sys
from llm_interface import LLMInterface
from geometric_parameterization import GeometricParameterizer
from trajectory_generation import TrajectoryGenerator
from visualization import ResultVisualizer


def main():
    """Main workflow: Geometric Parameterization -> Trajectory Generation"""
    
    print("=" * 60)
    print("EmbodiedCoder Demo")
    print("Geometric Parameterization + Trajectory Generation")
    print("=" * 60)
    
    try:
        # 1. Initialize LLM interface
        print("\n[Step 1] Initializing LLM interface...")
        llm = LLMInterface()
        
        # 2. Create geometric parameterizer and trajectory generator
        print("\n[Step 2] Creating geometric parameterizer and trajectory generator...")
        parameterizer = GeometricParameterizer(llm)
        generator = TrajectoryGenerator(llm)
        
        # 3. Define point clouds and tasks to process
        point_cloud_dir = "point_cloud"
        
        tasks = {
            'teacup': {
                'files': ['teacup.ply'],
                'task': 'Pick up the cup by the handle and lift it 20cm upward'
            },
            'tomato': {
                'files': ['tomato.ply'],
                'task': 'Grasp the tomato from the top and lift it gently upward'
            },
            'cookie_box': {
                'files': ['cookie_box.ply'],
                'task': 'Grasp the box from the sides and slide it forward 30cm'
            },
            'laptop': {
                'files': ['laptop_base.ply', 'laptop_screen.ply'],
                'task': 'Push the laptop screen to fold the screen over the keyboard'
            },
            'bowl': {
                'files': ['bowl.ply'],
                'task': 'Grasp the bowl at the RIM EDGE from the side (where humans naturally hold bowls - at the opening edge, NOT at the dome bottom). The rim contact point is at center + horizontal_offset * radius, where center is at the opening plane. Lift 15cm upward and move to new location.'
            }       
        }
        
        # Create output directories
        params_dir = "geometric_parameters"
        trajectory_dir = "trajectories"
        os.makedirs(params_dir, exist_ok=True)
        os.makedirs(trajectory_dir, exist_ok=True)
        
        # 4. Process each object
        all_results = {}
        
        for object_name, task_info in tasks.items():
            point_cloud_files = task_info['files']
            task_description = task_info['task']
            point_cloud_paths = [os.path.join(point_cloud_dir, f) for f in point_cloud_files]
            
            # Check if files exist
            missing = [p for p in point_cloud_paths if not os.path.exists(p)]
            if missing:
                print(f"\nWarning: Point cloud files do not exist: {missing}")
                continue
            
            print("\n" + "=" * 60)
            print(f"Processing object: {object_name}")
            print("=" * 60)
            
            # 4.1 Geometric parameterization
            print(f"\n[Step 3.{len(all_results)+1}a] Geometric parameterization...")
            param_result = parameterizer.parameterize(point_cloud_paths, object_name)
            
            # Save geometric parameters (JSON + algorithm code)
            parameterizer.save_results(param_result, params_dir)
            
            print(f"\nExtracted geometric parameters:")
            for key, value in param_result['parameters'].items():
                print(f"  {key}: {value}")
            
            # 4.2 Trajectory generation
            print(f"\n[Step 3.{len(all_results)+1}b] Trajectory generation...")
            trajectory_result = generator.generate_trajectory(
                object_name,
                param_result['parameters'],
                task_description
            )
            
            # Save trajectory
            trajectory_output = os.path.join(trajectory_dir, f"{object_name}_trajectory.py")
            generator.save_trajectory(trajectory_result, trajectory_output)
            
            if trajectory_result['trajectory']:
                json_output = os.path.join(trajectory_dir, f"{object_name}_waypoints.json")
                generator.save_trajectory_waypoints(trajectory_result, json_output)
                
                print(f"\nGenerated trajectory waypoint count: {len(trajectory_result['trajectory'])}")
                print("First 3 waypoints:")
                for i, waypoint in enumerate(trajectory_result['trajectory'][:3]):
                    print(f"  Waypoint {i+1}:")
                    for key, value in waypoint.items():
                        print(f"    {key}: {value}")
            
            # Save results
            all_results[object_name] = {
                'parameterization': param_result,
                'trajectory': trajectory_result
            }
        
        # 5. Generate summary report
        print("\n" + "=" * 60)
        print("Processing Completion Summary")
        print("=" * 60)
        print(f"\nProcessed {len(all_results)} objects in total:")
        for obj_name in all_results.keys():
            print(f"  ✓ {obj_name}")
        
        print(f"\nOutput files:")
        print(f"  - Geometric parameters: {params_dir}/")
        print(f"  - Trajectory code: {trajectory_dir}/")
        
        # Generate README
        readme_path = "README_results.md"
        generate_readme(all_results, readme_path)
        print(f"  - Results description: {readme_path}")
        
        print("\n✓ All tasks completed!")
        
        # 6. Visualize results
        print("\n" + "=" * 60)
        print("Visualize Results")
        print("=" * 60)
        
        visualize_results = input("\nDo you want to visualize the results? (y/n, default y): ").strip().lower()
        if visualize_results != 'n':
            visualizer = ResultVisualizer()
            
            # Select visualization mode
            print("\nVisualization options:")
            print("1. Display each object individually")
            print("2. Display all objects simultaneously")
            choice = input("Please choose (1/2, default 2): ").strip()
            
            if choice == '1':
                # Visualize individually
                for object_name, result in all_results.items():
                    point_cloud_paths = result['parameterization']['point_cloud_paths']
                    params = result['parameterization']['parameters']
                    trajectory = result['trajectory'].get('trajectory')
                    
                    print(f"\nPress Enter to view {object_name}...")
                    input()
                    visualizer.visualize_single_result(
                        object_name, point_cloud_paths, params, trajectory
                    )
            else:
                # Visualize all objects simultaneously
                visualizer.visualize_all_results(all_results, point_cloud_dir)
        
        return all_results
        
    except Exception as e:
        print(f"\n✗ Error: {e}")
        import traceback
        traceback.print_exc()
        return None


def generate_readme(results, output_path):
    """Generate results documentation"""
    with open(output_path, 'w', encoding='utf-8') as f:
        f.write("# EmbodiedCoder Results Report\n\n")
        f.write("## Overview\n\n")
        f.write("This report shows the results of geometric parameterization and trajectory generation using LLM.\n\n")
        
        f.write("## Processed Objects\n\n")
        for obj_name, result in results.items():
            f.write(f"### {obj_name}\n\n")
            
            # Geometric parameters
            f.write("**Geometric Parameters:**\n\n")
            params = result['parameterization']['parameters']
            for key, value in params.items():
                f.write(f"- `{key}`: {value}\n")
            
            # Task description
            f.write(f"\n**Task Description:** {result['trajectory']['task']}\n\n")
            
            # Trajectory information
            if result['trajectory']['trajectory']:
                f.write(f"**Trajectory Waypoint Count:** {len(result['trajectory']['trajectory'])}\n\n")
            
            f.write("---\n\n")
        
        f.write("## File Structure\n\n")
        f.write("```\n")
        f.write("geometric_parameters/     # Geometric parameter code\n")
        f.write("  ├── teacup_params.py\n")
        f.write("  ├── tomato_params.py\n")
        f.write("  ├── cookie_box_params.py\n")
        f.write("  └── laptop_params.py\n")
        f.write("\n")
        f.write("trajectories/            # Trajectory code and waypoints\n")
        f.write("  ├── teacup_trajectory.py\n")
        f.write("  ├── teacup_waypoints.json\n")
        f.write("  ├── tomato_trajectory.py\n")
        f.write("  ├── tomato_waypoints.json\n")
        f.write("  ├── cookie_box_trajectory.py\n")
        f.write("  ├── cookie_box_waypoints.json\n")
        f.write("  ├── laptop_trajectory.py\n")
        f.write("  └── laptop_waypoints.json\n")
        f.write("```\n\n")
        
        f.write("## Usage\n\n")
        f.write("1. View geometric parameters in `geometric_parameters/`\n")
        f.write("2. View trajectory code in `trajectories/`\n")
        f.write("3. Use JSON files to import trajectory waypoints into robot control system\n")


if __name__ == "__main__":
    main()
