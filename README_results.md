# EmbodiedCoder Results Report

## Overview

This report shows the results of geometric parameterization and trajectory generation using LLM.

## Processed Objects

### teacup

**Geometric Parameters:**

- `parameters`: {'component_0': {'type': 'cylinder', 'center': [0.5128707412430444, 0.2786786828568041, 0.05331613848161611], 'radius': 0.034870785204393726, 'height': 0.08597257473249452, 'axis': [0.22690922686343973, -0.46018672000008387, 0.8583358232648807]}}

**Task Description:** Pick up the cup by the handle and lift it 20cm upward

**Trajectory Waypoint Count:** 7

---

### tomato

**Geometric Parameters:**

- `parameters`: {'component_0': {'type': 'ellipsoid', 'center': [0.000381228912770204, -0.0005386157871417295, 0.008084744316031803], 'radii': [0.04277209249829337, 0.043920959661371405, 0.04371548390764809]}}

**Task Description:** Grasp the tomato from the top and lift it gently upward

**Trajectory Waypoint Count:** 7

---

### cookie_box

**Geometric Parameters:**

- `parameters`: {'component_0': {'type': 'box', 'center': [0.0, 0.0, 0.04023660452591979], 'dimensions': [0.153, 0.10200000000000001, 0.08111732676712363]}}

**Task Description:** Grasp the box from the sides and slide it forward 30cm

**Trajectory Waypoint Count:** 7

---

### laptop

**Geometric Parameters:**

- `parameters`: {'base_component_0': {'type': 'box', 'center': [0.0, 7.906915667603431e-05, 0.010017607076819009], 'dimensions': [0.306, 0.2038386989203809, 0.020364081563289223]}, 'screen_component_0': {'type': 'box', 'center': [-0.0004104585458583193, -0.015244887992938896, 0.06604663538652548], 'dimensions': [0.3065137372390881, 0.21411089429574653, 0.005712742885258286], 'rotation_matrix': [[0.9997847940921345, -0.020737206948387436, -0.000577711111286072], [0.017644306712299072, 0.8646560727814748, -0.5020543339547621], [0.010910726043241555, 0.501936095583987, 0.8648358873260941]]}}

**Task Description:** Push the laptop screen to fold the screen over the keyboard

**Trajectory Waypoint Count:** 6

---

### bowl

**Geometric Parameters:**

- `parameters`: {'component_0': {'type': 'hemisphere', 'center': [2.48412142726806e-06, -0.0018064078770194322, 0.03473984722707365], 'radius': 0.1077085153193018, 'axis': [0.003410643858209094, 0.09382289490514584, 0.9955830612762003]}}

**Task Description:** Grasp the bowl at the RIM EDGE from the side (where humans naturally hold bowls - at the opening edge, NOT at the dome bottom). The rim contact point is at center + horizontal_offset * radius, where center is at the opening plane. Lift 15cm upward and move to new location.

**Trajectory Waypoint Count:** 8

---

## File Structure

```
geometric_parameters/     # Geometric parameter code
  ├── teacup_params.py
  ├── tomato_params.py
  ├── cookie_box_params.py
  └── laptop_params.py

trajectories/            # Trajectory code and waypoints
  ├── teacup_trajectory.py
  ├── teacup_waypoints.json
  ├── tomato_trajectory.py
  ├── tomato_waypoints.json
  ├── cookie_box_trajectory.py
  ├── cookie_box_waypoints.json
  ├── laptop_trajectory.py
  └── laptop_waypoints.json
```

## Usage

1. View geometric parameters in `geometric_parameters/`
2. View trajectory code in `trajectories/`
3. Use JSON files to import trajectory waypoints into robot control system
