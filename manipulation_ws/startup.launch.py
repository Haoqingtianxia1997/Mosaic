#  Copyright (c) 2023 Franka Robotics GmbH
#
#  Licensed under the Apache License, Version 2.0 (the "License");
#  you may not use this file except in compliance with the License.
#  You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
#  Unless required by applicable law or agreed to in writing, software
#  distributed under the License is distributed on an "AS IS" BASIS,
#  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#  See the License for the specific language governing permissions and
#  limitations under the License.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    
    return LaunchDescription([
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource([PathJoinSubstitution(
        #         [FindPackageShare('franka_bringup'), 'launch', 'franka.launch.py'])]),
        #     launch_arguments={
        #                       'robot_ip':'192.168.2.55',
        #                       'load_gripper': 'true',
        #                       'use_rviz': 'false',
        #                     #   'arm_id': 'fr3'
        #                       }.items(),
        # ),
        
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource([PathJoinSubstitution(
        #         [FindPackageShare('franka_moveit_config'), 'launch', 'moveit_new.launch.py'])]),
        # ),
        
        # ros2 launch zed_wrapper zed_camera.launch.py camera_name:=zedr camera_model:=zed2 serial_number:=21177909 publish_urdf:=true publish_tf:=false publish_map_tf:=false publish_imu_tf:=false

        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource([PathJoinSubstitution(
        #         [FindPackageShare('zed_wrapper'), 'launch', 'zed_camera.launch.py'])]),
        #     launch_arguments={
        #                         'camera_name':'zedr',
        #                         'camera_model':'zed2',
        #                         'serial_number':'21177909',
        #                         'publish_urdf':'true',
        #                         'publish_tf':'false',
        #                         'publish_map_tf': 'false',
        #                         'publish_imu_tf': 'false',
        #                       }.items(),
        # ),
        
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource([PathJoinSubstitution(
        #         [FindPackageShare('zed_wrapper'), 'launch', 'zed_camera.launch.py'])]),
        #     launch_arguments={
        #                         'camera_name':'zedl',
        #                         'camera_model':'zed2',
        #                         'serial_number':'29934236',
        #                         'publish_urdf':'true',
        #                         'publish_tf':'false',
        #                         'publish_map_tf': 'false',
        #                         'publish_imu_tf': 'false',
        #                       }.items(),
        # ),
        
        # Node(
        #     package='rviz2',
        #     executable='rviz2',
        #     name='rviz2',
        #     output='screen',
        #     arguments=['-d', PathJoinSubstitution([FindPackageShare('ursa_ros'), 'launch', 'display.rviz'])]
        # ),
        
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_publisher',
            output='screen',
            arguments=[
                '--x', '0.8595406021203789',
                '--y', '0.5037965577635406',
                '--z', '0.5703258113488032',
                '--qx', '0.37301026915950647',
                '--qy', '0.06207915901055871',
                '--qz', '-0.8924183137659721',
                '--qw', '0.24616878431920047',
                '--frame-id', 'panda_link0',
                '--child-frame-id', 'zedr_camera_link'
            ]
        ),
        
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_publisher_2',
            output='screen',
            arguments=[
                '--x', '0.14287264529286604',
                '--y', '-0.5160394374662502',
                '--z', '0.5285747070512808',
                '--yaw', '1.0319360545389382',
                '--pitch', '0.8017128013264066',
                '--roll', '0.4719635237773772',
                '--frame-id', 'panda_link0',
                '--child-frame-id', 'zedl_camera_link'
            ]
        ),
        
        # Node(
        #     package='usb_cam',
        #     executable='usb_cam_node_exe',
        #     name='external_camera',
        #     output='screen',
        #     arguments=[
        #         '--ros-args', '--params-file', '/home/vignesh/ros2_ws/src/ursa_ros/config/usb_cam_params.yaml',
        #     ]
        # )
    ])
