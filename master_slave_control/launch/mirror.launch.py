from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory
import rclpy

def generate_launch_description():
    # RViz launch for RX150 (master with joint GUI)
    rviz_launch_rx150 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('interbotix_xsarm_descriptions'),
                'launch', 'xsarm_description.launch.py'
            ),
        ),
        launch_arguments={
            'robot_model': 'rx150',
            'robot_name': 'rx150',
            'use_joint_pub_gui': 'true'
        }.items()
    )

    # RViz launch for VX300 (slave without GUI)
    rviz_launch_vx300 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('interbotix_xsarm_descriptions'),
                'launch', 'xsarm_description.launch.py'
            ),
        ),
        launch_arguments={
            'robot_model': 'vx300',
            'robot_name': 'vx300',
            'use_joint_pub_gui': 'false'
        }.items()
    )

    print("Launching File!")

    return LaunchDescription([
        rviz_launch_rx150,
        rviz_launch_vx300,
        # Node(
        #     package='master_slave_control',
        #     executable='mirror_nodes',
        #     name='mirror_nodes',
        #     output='screen',
        #     parameters=[{'robot_name': 'rx150'}, {'robot_name': 'vx300'}],
        # ),
    ])
