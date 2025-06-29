## Prerequisites
```bash
sudo iptables -I INPUT 1 -s 192.168.2.55 -j ACCEPT
ros2 launch franka_bringup gravity_compensation_example_controller.launch.py robot_ip:=192.168.2.55 use_rviz:=True
ros2 launch franka_moveit_config moveit_new.launch.py
```

## move
```bash
ros2 topic pub --once /panda_arm_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory "
joint_names: ['panda_joint1', 'panda_joint2', 'panda_joint3', 'panda_joint4', 'panda_joint5', 'panda_joint6', 'panda_joint7']
points:
- positions: [0.0, 0.0, 0.0, -1.57, 0.0, 1.57, 0.0]
  time_from_start: {sec: 3, nanosec: 0}
"

ros2 run action move_service_default 0.4 0.3 0.15 1.0 0.0 0.0 0.0

ros2 run action move_service_default 0.6 0.0 0.5 1.0 0.0 0.0 0.0
```

# gripper
```bash
ros2 action send_goal /panda_gripper/move franka_msgs/action/Move "{width: 0.08, speed: 0.1}"
```

```bash
ros2 action send_goal /panda_gripper/move franka_msgs/action/Move "{width: 0.01, speed: 0.1}" 
```

```bash
ros2 action send_goal /panda_gripper/grasp franka_msgs/action/Grasp "{width: 0.01, speed: 0.1, force: 20.0, epsilon: {inner: 0.01, outer: 0.01}}"
```

# vision
```bash
# right camera
ros2 launch zed_wrapper zed_camera.launch.py camera_name:=zedr camera_model:=zed2 serial_number:=21177909 publish_urdf:=true publish_tf:=false publish_map_tf:=false publish_imu_tf:=false
# left camera
ros2 launch zed_wrapper zed_camera.launch.py camera_name:=zedl camera_model:=zed2 serial_number:=29934236 publish_urdf:=true publish_tf:=false publish_map_tf:=false publish_imu_tf:=false

# image service node for left camera
ros2 run action image_saver
```


# finished
move_service:
```bash
ros2 run action move
ros2 service call /move_service action_interfaces/srv/Move "{x: 0.5, y: 0.0, z: 0.5, qx: 1.0, qy: 0.0, qz: 0.0, qw: 0.0}"
```

move_cartesian:
```bash
ros2 run action move_cartesian
ros2 service call /move_cartesian_service action_interfaces/srv/Move "{x: 0.4, y: 0.3, z: 0.3, qx: 1.0, qy: 0.0, qz: 0.0, qw: 0.0}"
```

close:
```bash
ros2 run action close
ros2 service call /close_service std_srvs/srv/Trigger "{}"
```

open:
```bash
ros2 run action open
ros2 service call /open_service std_srvs/srv/Trigger "{}"
```

reset:
```bash
ros2 run action reset
ros2 service call /reset_service std_srvs/srv/Trigger
```

stir:
```bash
ros2 run action stir
ros2 service call /stir_service action_interfaces/srv/Stir "{center_x: 0.6, center_y: -0.3, center_z: 0.4, radius: 0.08, start_angle_deg: 0.0, move_down_offset: 0.1, speed: 0.5, stir_time: 30}"
```


add:
```bash
ros2 run action add
ros2 service call /add_service action_interfaces/srv/Add "{times: 2}"
```

grasp:
```bash
ros2 run action grasp
ros2 service call /grasp_service action_interfaces/srv/Grasp "{
  x_prep: 0.5,
  y_prep: 0.6,
  z_prep: 0.3,
  qx_prep: 1.0,
  qy_prep: 0.0, 
  qz_prep: 0.0,
  qw_prep: 0.0,
  x_grasp: 0.5,
  y_grasp: 0.6,
  z_grasp: 0.2,
  qx_grasp: 1.0,
  qy_grasp: 0.0,
  qz_grasp: 0.0,
  qw_grasp: 0.0
}"
```

return_back:
```bash
ros2 run action return_back
ros2 service call /return_back_service action_interfaces/srv/ReturnBack "{
  x_prep: 0.5,
  y_prep: 0.0,
  z_prep: 0.5,
  qx_prep: 1.0,
  qy_prep: 0.0, 
  qz_prep: 0.0,
  qw_prep: 0.0,
  x_place: 0.5,
  y_place: 0.0,
  z_place: 0.4,
  qx_place: 1.0,
  qy_place: 0.0,
  qz_place: 0.0,
  qw_place: 0.0
}"
```

launch:
```bash
ros2 launch action mosaic_launch