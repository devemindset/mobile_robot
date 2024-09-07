import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Define package and file paths
    package_name = 'mobile_robot_v1'  # Replace with your package name
    xacro_file_name = 'robot_mobile.urdf.xacro'  # Replace with your xacro file name
    rviz_config_file_name = 'conf_chassis.rviz'  # Replace with your rviz config file name if available

    # Paths
    xacro_file_path = os.path.join(get_package_share_directory(package_name), 'urdf', xacro_file_name)
    rviz_config_file_path = os.path.join(get_package_share_directory(package_name), 'rviz', rviz_config_file_name)


    # Declare the robot_description parameter
    robot_description = Command(['xacro ', xacro_file_path])




    return LaunchDescription([
        # RViz node
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_file_path],
            parameters=[{'robot_description': robot_description}],
        )
    ])