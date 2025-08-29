#!/bin/bash

source /opt/ros/humble/setup.bash
source ~/manipulation_ws/install/setup.bash

sudo iptables -I INPUT 1 -s 192.168.2.55 -j ACCEPT

SCRIPT_DIR=$(cd $(dirname $0) && pwd)

SESSION="mosaic"

# session 1: moveit2
tmux new-session -d -s $SESSION -n moveit "bash -c 'cd $SCRIPT_DIR && ros2 launch franka_fr3_moveit_config moveit.launch.py robot_ip:=192.168.2.55'"

# session 2: right camera
tmux new-window -t $SESSION:1 -n cam_r "bash -c 'cd $SCRIPT_DIR && ros2 launch zed_wrapper zed_camera.launch.py camera_name:=zedr camera_model:=zed2 serial_number:=21177909 publish_urdf:=true publish_tf:=false publish_map_tf:=false publish_imu_tf:=false'"

# session 3: left camera
tmux new-window -t $SESSION:2 -n cam_l "bash -c 'cd $SCRIPT_DIR && ros2 launch zed_wrapper zed_camera.launch.py camera_name:=zedl camera_model:=zed2 serial_number:=29934236 publish_urdf:=true publish_tf:=false publish_map_tf:=false publish_imu_tf:=false'"

# session 4: gaze camera
tmux new-window -t $SESSION:3 -n cam_gaze "bash -c 'cd $SCRIPT_DIR && ros2 launch realsense2_camera rs_launch.py enable_rgbd:=true enable_sync:=true align_depth.enable:=true depth_module.depth_profile:=848x480x30 rgb_camera.color_profile:=848x480x30'"

# session 5: launch
tmux new-window -t $SESSION:4 -n launch "bash -c 'cd $SCRIPT_DIR/manipulation_ws && ros2 launch action mosaic_launch.py'"

# session 6: intention detection
tmux new-window -t $SESSION:5 -n intent_detect "bash -c 'cd $SCRIPT_DIR/manipulation_ws && python3 src/action/action/intention_detection.py'"

# session 7: intention llm
tmux new-window -t $SESSION:6 -n intent_llm "bash -c 'cd $SCRIPT_DIR/manipulation_ws && python3 src/action/action/intention_llm.py'"

# session 8: high level main
tmux new-window -t $SESSION:7 -n high_level "bash -c 'cd $SCRIPT_DIR/high_level && python3 1.main.py'"

# Attach to the tmux session
tmux attach -t $SESSION
