from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
def generate_launch_description():
    return LaunchDescription([
        Node(
            package='action',
            executable='move',
            name='move_node',
            output='screen',
        ),
        Node(
            package='action',
            executable='move_cartesian',
            name='move_cartesian_node',
            output='screen',
        ),
        Node(
            package='action',
            executable='close',
            name='close_node',
            output='screen',
        ),
        Node(
            package='action',
            executable='open',
            name='open_node',
            output='screen',
        ),
        Node(
            package='action',
            executable='add',
            name='add_node',
            output='screen',
        ),
        Node(
            package='action',
            executable='grasp',
            name='grasp_node',
            output='screen',
        ),
        Node(
            package='action',
            executable='image_saver',
            name='image_saver_node',
            output='screen',
        ),
        Node(
            package='action',
            executable='reset',
            name='reset_node',
            output='screen',
        ),
        Node(
            package='action',
            executable='return_back',
            name='return_back_node',
            output='screen',
        ),
        Node(
            package='action',
            executable='stir',
            name='stir_node',
            output='screen',
        ),

        Node(
            package='action',
            executable='forward_kinematic',
            name='forward_kinematic',
            output='screen',
        ),

        # ExecuteProcess(
        #     cmd=[
        #         '/home/mosaic/miniconda3/envs/mosaic/bin/python',
        #         '/home/mosaic/mosaic/manipulation_ws/src/action/action/intention_detection.py'
        #     ],
        #     name='intention_detection_process',
        #     output='screen'
        # ),
    ])
