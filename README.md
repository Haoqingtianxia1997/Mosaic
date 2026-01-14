# EmbodiedCoder Implementation

**Geometric Parameterization and Trajectory Generation System Using Large Language Models**

## Overview

This project implements an LLM-based system for robotic manipulation that automatically:
1. **Converts point clouds to geometric parameters** - Fits primitive shapes (cylinders, boxes, spheres, hemispheres, ellipsoids) to 3D point cloud data
2. **Generates manipulation trajectories** - Creates robot waypoints based on geometric understanding and human manipulation habits
3. **Visualizes results** - Shows point clouds, fitted geometries, and trajectories in 3D

**Key Innovation**: Uses GPT-4o to generate both the parameterization algorithms AND visualization code, making the system adaptable to diverse object shapes.

## Features

✅ **5 Object Types**: Tea cup, bowl, tomato, cookie box, laptop  
✅ **6 Geometric Primitives**: Box, cylinder, sphere, hemisphere, ellipsoid, torus  
✅ **Robust Geometry Fitting**: Rotation-invariant hemisphere detection, proper rotation matrix handling  
✅ **Human-Centric Trajectories**: Contact points based on natural human grasping habits  
✅ **Automatic Code Generation**: LLM generates extraction algorithms and visualization functions  
✅ **Full Visualization**: Point clouds, transparent geometries, and trajectory waypoints in Open3D  

## System Architecture

```
├── llm_interface.py              # LLM interface module (OpenAI API)
├── geometric_parameterization.py # Geometric parameterization module
├── trajectory_generation.py      # Trajectory generation module
├── visualization.py              # Visualization module
├── main.py                       # Main program
├── point_cloud_generation.py     # Point cloud generation
├── point_cloud/                  # Point cloud data directory
│   ├── teacup.ply               #   Tea cup with tilted orientation
│   ├── bowl.ply                 #   Bowl (hemisphere geometry)
│   ├── tomato.ply               #   Tomato (ellipsoid)
│   ├── cookie_box.ply           #   Cookie box (rectangular)
│   ├── laptop.ply               #   Laptop (base + tilted screen)
│   ├── laptop_base.ply          #   Laptop base component
│   └── laptop_screen.ply        #   Laptop screen component
├── geometric_parameters/         # Geometric parameter extraction results
│   ├── *_params.json            #   Geometric parameters (JSON)
│   └── *_algorithm.py           #   LLM-generated extraction & visualization code
└── trajectories/                 # Robot manipulation trajectory results
    ├── *_trajectory.py          #   LLM-generated trajectory code
    └── *_waypoints.json         #   Trajectory waypoints data
```

## Dependency Installation

```bash
# Install Python dependencies
pip install openai numpy open3d matplotlib scikit-learn
```

## Quick Start

```bash
# 1. Set your OpenAI API key
$env:OPENAI_API_KEY="your-api-key-here"

# 2. Generate point clouds (if not already generated)
python point_cloud_generation.py

# 3. Run the complete pipeline
python main.py
```

The system will:
1. Load all point cloud files from `point_cloud/` directory
2. Generate geometric parameters and save to `geometric_parameters/`
3. Generate manipulation trajectories and save to `trajectories/`
4. Offer to visualize results (individual or all objects side-by-side)

## Environment Configuration

Set OpenAI API key:

**Windows (PowerShell):**
```powershell
$env:OPENAI_API_KEY="your-api-key-here"
```

**Windows (Permanent Setting):**
```powershell
[System.Environment]::SetEnvironmentVariable('OPENAI_API_KEY', 'your-api-key-here', 'User')
```

**Linux/Mac:**
```bash
export OPENAI_API_KEY="your-api-key-here"
```

## Usage

### 1. Generate Point Clouds (if not already generated)

```bash
python point_cloud_generation.py
```

This will generate point cloud files for **5 objects** in the `point_cloud/` directory:
- **Teacup**: Cylinder body + ring handle, with rotation (15° X-axis, 10° Y-axis) and translation
- **Bowl**: Hemisphere geometry (half-sphere with opening)
- **Tomato**: Ellipsoid body + small cylindrical stem
- **Cookie Box**: Rectangular box
- **Laptop**: Base box + tilted screen (simulating open laptop)

### 2. Run Complete Pipeline

```bash
python main.py
```

This will execute:
1. Load point cloud files
2. Perform geometric parameterization for each object
3. Generate manipulation trajectories based on geometric parameters
4. Save all results
5. **Visualize point clouds, parameterized geometries and trajectories**

The program will ask whether to visualize, with options:
- Display each object individually (press Enter to switch)
- Display all objects simultaneously (side-by-side display)

### 3. Test Individual Modules (if you need)

**Test LLM Interface:**
```bash
python llm_interface.py
```

**Test Geometric Parameterization:**
```bash
python geometric_parameterization.py
```

**Test Trajectory Generation:**
```bash
python trajectory_generation.py
```

**Test Visualization:**
```bash
python visualization.py
```

**Visualize a Specific Object:**
```python
from visualization import visualize_from_files

# Visualize bowl with fitted hemisphere geometry and trajectory
visualize_from_files(
    object_name="bowl",
    point_cloud_path="point_cloud/bowl.ply",
    params_file="geometric_parameters/bowl_params.json",
    waypoints_file="trajectories/bowl_waypoints.json"
)

# Visualize tilted teacup with cylinder and handle
visualize_from_files(
    object_name="teacup",
    point_cloud_path="point_cloud/teacup.ply",
    params_file="geometric_parameters/teacup_params.json",
    waypoints_file="trajectories/teacup_waypoints.json"
)
```

## Output Files

After running, the following directories and files will be generated:

```
geometric_parameters/          # Geometric parameters
├── teacup_params.json        # Tea cup parameters (cylinder + torus handle)
├── teacup_algorithm.py       # Extraction & visualization code
├── bowl_params.json          # Bowl parameters (hemisphere)
├── bowl_algorithm.py         # Extraction & visualization code
├── tomato_params.json        # Tomato parameters (ellipsoid + stem)
├── tomato_algorithm.py       # Extraction & visualization code
├── cookie_box_params.json    # Cookie box parameters (box)
├── cookie_box_algorithm.py   # Extraction & visualization code
├── laptop_params.json        # Laptop parameters (base + rotated screen)
└── laptop_algorithm.py       # Extraction & visualization code

trajectories/                  # Trajectory code and data
├── teacup_trajectory.py      # Tea cup manipulation trajectory code
├── teacup_waypoints.json     # Tea cup waypoints
├── bowl_trajectory.py        # Bowl manipulation trajectory code
├── bowl_waypoints.json       # Bowl waypoints
├── tomato_trajectory.py      # Tomato manipulation trajectory code
├── tomato_waypoints.json     # Tomato waypoints
├── cookie_box_trajectory.py  # Cookie box manipulation trajectory code
├── cookie_box_waypoints.json # Cookie box waypoints
├── laptop_trajectory.py      # Laptop manipulation trajectory code
└── laptop_waypoints.json     # Laptop waypoints

README_results.md             # Results summary report
```

## Core Features

### 1. Geometric Parameterization (geometric_parameterization.py)

- **Input**: Point cloud files (.ply format)
- **Processing**: 
  - Analyze point cloud statistical features (centroid, dimensions, principal components, etc.)
  - Use LLM to automatically select appropriate geometric primitives
  - Generate algorithm code to extract geometric parameters
  - Generate visualization functions for the fitted geometry
- **Output**: 
  - Geometric parameters (JSON file)
  - Algorithm code (Python file with extraction and visualization functions)

**Supported Geometric Primitives**:
- **Box**: Rectangular prism, with optional rotation matrix for oriented boxes
- **Cylinder**: Circular cylinder with height axis direction
- **Sphere**: Complete closed sphere
- **Hemisphere**: Half-sphere with flat circular base (e.g., bowls)
  - Critical: axis points FROM dome center TOWARD opening (direction to look inside)
  - Robust direction detection using XY radius comparison
- **Ellipsoid**: Ellipsoid with three different semi-axes
- **Torus**: Donut/ring shape (e.g., cup handles)

**Key Improvements**:
- **Hemisphere Direction Detection**: Uses XY radius analysis (opening side has larger radius) instead of point distribution, making it rotation-invariant
- **Cylinder Axis**: Uses first principal component from PCA for accurate height axis
- **Rotated Objects**: Proper handling of rotation matrices for tilted objects (e.g., teacup, laptop screen)

**System Prompt**: Guides LLM to identify geometric primitives and extract key parameters with detailed examples

### 2. Trajectory Generation (trajectory_generation.py)

- **Input**: Geometric parameters + task description
- **Processing**:
  - Generate manipulation trajectories based on geometric parameters and task description
  - Include approach, grasp, manipulation, and retreat phases
  - Consider collision avoidance and smooth motion
  - **Human-Centric Analysis**: Contact points based on how humans naturally grasp objects
- **Output**: 
  - Trajectory generation code (Python function)
  - Waypoint data (JSON format)

**Human Manipulation Habit Analysis**:
- **Bowl/Cup (Hemisphere/Cylinder)**: Grasp at RIM/EDGE from the side (fingers wrap around rim opening)
- **Box/Package**: Grasp at EDGES or CORNERS for leverage
- **Sphere/Ball**: Grasp at EQUATOR (midsection) where hand wraps naturally
- **Cylinder (vertical)**: Grasp at MID-HEIGHT for balance

**Contact Point Calculation Rules**:
- **Hemisphere (Bowl)**: Contact at rim edge = center + horizontal_direction × radius × 0.9
  - Critical: center is at opening plane, NOT dome center
  - Grasp where humans hold bowls - at opening edge, NOT at bottom!
- **Cylinder**: Contact at center ± [radius × cos(angle), radius × sin(angle), z_position]
- **Box (rotated)**: Transform local offset to world frame using rotation matrix
- **Sphere/Ellipsoid**: Contact at center ± radius × direction_vector

**System Prompt**: Guides LLM to generate complete trajectories with proper surface contact based on human manipulation habits

### 3. Visualization (visualization.py)

- **Features**:
  - Display point clouds in Open3D
  - Create parameterized geometries based on geometric parameters (semi-transparent)
  - Visualize robot manipulation trajectories (waypoints and connections)
  - Support single or multiple objects displayed side-by-side

- **Visualization Content**:
  - **Point Cloud**: Original scanned data
  - **Geometries**: Parameterized semi-transparent 3D models (cylinders, spheres, boxes, etc.)
  - **Trajectories** (Unified Color Scheme):
    - 🟢 **Green large sphere**: Start point (all trajectories)
    - 🔴 **Red large sphere**: End point (all trajectories)
    - 🔵 **Blue small spheres**: Intermediate waypoints
    - 🔵 **Blue line**: Connecting all waypoints
  - **Coordinate Frame**: RGB axes showing orientation

### 4. LLM Interface (llm_interface.py)

- Encapsulates OpenAI API calls
- Supports custom system prompts and temperature parameters
- Automatic code block extraction
- Error handling

## Paper Correspondence

### Geometric Parameterization Phase

Paper's Method:
1. Analyze point cloud features (centroid, bounds, principal components)
2. Use LLM to convert geometric information to parameterized description
3. Extract key geometric parameters for subsequent planning

This Implementation:
- `GeometricParameterizer.select_primitives_with_llm()`: Automatic primitive selection
- `GeometricParameterizer.assemble_prompt()`: Construct prompt
- System prompt guides LLM to identify geometric primitives and parameters
- LLM generates both extraction algorithm and visualization function

### Trajectory Generation Phase

Paper's Method:
1. Generate manipulation trajectories based on geometric parameters
2. Use LLM to generate complete trajectories with multiple phases
3. Output executable waypoint sequences

This Implementation:
- `TrajectoryGenerator.create_trajectory_prompt()`: Construct trajectory generation prompt
- `TrajectoryGenerator.generate_trajectory()`: Generate and execute trajectory code
- System prompt guides LLM to generate structured waypoints

## Key Design

1. **Code-Driven**: LLM generates executable Python code, not pure text descriptions
2. **Modular**: Geometric parameterization and trajectory generation are separated for easy debugging and extension
3. **Verifiable**: Uses real point cloud data to validate the entire pipeline
4. **Structured Output**: Generated code has clear data structures (dictionaries, lists)
5. **LLM-Generated Visualization**: Visualization functions are automatically generated by LLM based on geometry
6. **Human-Centric**: Trajectory generation considers how humans naturally interact with objects
7. **Robust Geometry Fitting**: 
   - Hemisphere direction detection using XY radius analysis (rotation-invariant)
   - Proper rotation matrix handling for tilted objects
   - Clear semantic definitions for geometry axes

## Technical Highlights

### Hemisphere (Bowl) Geometry Handling
- **Challenge**: Detecting opening direction regardless of bowl orientation
- **Solution**: Compare XY radius of top vs bottom half (opening side has larger radius)
- **Advantage**: Works even when bowl is inverted or tilted
- **Axis Definition**: Points FROM dome center TOWARD opening (direction to look inside)

### Rotated Object Handling
- **Teacup**: Rotated 15° on X-axis and 10° on Y-axis with translation [0.05, 0.03, 0.02]
- **Laptop Screen**: Tilted screen with rotation matrix representing opening angle
- **Contact Points**: Transform local offsets to world frame using rotation matrix

### Human Manipulation Habits
- **Analysis Phase**: Before generating trajectory, analyze WHERE humans naturally grasp
- **Contact Rules**: Different primitives have different natural grasp points
- **Bowl Example**: Humans grasp at rim edge (opening), NOT at dome bottom
- **Result**: Generated trajectories that mimic human manipulation patterns

## Extension Directions

1. **More Object Types**: Add more complex geometries (cones, partial cylinders, composite shapes)
2. **Real Robot Integration**: Connect to real robot control interfaces (ROS, MoveIt)
3. **Advanced Planning**: Add trajectory optimization and collision detection
4. **Multimodal Input**: Support images + point clouds for richer context
5. **Feedback Loops**: Add trajectory adjustment based on execution results
6. **Enhanced Visualization**: Animation playback, interactive editing, real-time parameter tuning
7. **Learning from Demonstrations**: Incorporate human demonstration data for better manipulation habits
8. **Dynamic Objects**: Handle deformable or articulated objects (e.g., fabric, multi-link robots)

## Visualization Instructions

### Open3D Interactive Controls

In the visualization window:
- **Left mouse drag**: Rotate view
- **Mouse wheel**: Zoom
- **Right mouse drag**: Pan view
- **R key**: Reset view
- **+/- keys**: Increase/decrease point size

### Visualization Elements

1. **Point Clouds** (original colors)
   - Teacup: Light blue (tilted 15° on X-axis, 10° on Y-axis)
   - Bowl: Light blue (hemisphere with opening upward)
   - Tomato: Red body + green stem
   - Cookie box: Golden yellow
   - Laptop: Dark gray

2. **Parameterized Geometries** (semi-transparent)
   - Teacup: Cylinder + ring handle
   - Bowl: Hemisphere (half-sphere)
   - Tomato: Ellipsoid + small stem
   - Cookie box: Rectangular box
   - Laptop: Base + tilted screen

3. **Trajectories** (Unified Color Scheme)
   - 🔵 **Blue line**: Connecting all waypoints
   - 🟢 **Green large sphere**: Start point
   - 🔴 **Red large sphere**: End point
   - 🔵 **Blue small spheres**: Intermediate waypoints

4. **Coordinate Frame**
   - Red axis: X-axis
   - Green axis: Y-axis
   - Blue axis: Z-axis

## Troubleshooting

**Issue: openai module not found**
```bash
pip install openai --upgrade
```

**Issue: API key error**
- Check if environment variable is set correctly
- Confirm API key is valid and has balance

**Issue: Point cloud files do not exist**
- Run `python point_cloud_generation.py` first to generate point clouds

**Issue: Generated code cannot be executed**
- Check code format in LLM response
- Try adjusting temperature parameter (lower for more deterministic output)

**Issue: Visualization geometry not aligned with point cloud**
- The LLM-generated algorithm should handle coordinate transformations correctly
- Check the algorithm code in `geometric_parameters/` directory
- For rotated objects (teacup, laptop), ensure rotation_matrix is properly applied
- For hemispheres (bowl), verify axis direction points from dome to opening

**Issue: Trajectory waypoints not contacting object surface**
- Check if geometric parameters are correct (radius, dimensions, center)
- Verify that human manipulation habit analysis is included in task description
- For bowls, ensure grasp points are at rim edge (center + horizontal_offset × radius)
- Review contact point calculation in generated trajectory code

**Issue: Hemisphere opening direction inverted**
- The robust XY radius method should handle this automatically
- Verify that opening side has larger XY radius than dome side
- Check bowl_algorithm.py for the direction detection logic

## Reference

Based on paper: EmbodiedCoder - Geometric Parameterization and Trajectory Generation for Robot Manipulation Using Large Language Models
