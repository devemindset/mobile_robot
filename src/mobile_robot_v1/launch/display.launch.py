import launch
from launch.substitutions import Command, LaunchConfiguration 
import launch_ros
import os 

def generate_launch_description():
    pkg_share = launch_ros.substitutions.FindPackageShare(package="mobile_robot_v1").find("mobile_robot_v1")
    default_model_path = os.path.join(pkg_share,"urdf/robot_mobile.urdf.xacro")
    default_rviz_config_path = os.path.join(pkg_share,"rviz/conf_chassis.rviz")

    robot_state_publisher_node = launch_ros.actions.Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[ {"robot_description" : Command(["xacro ", LaunchConfiguration("model")])}]
    )
    join_state_publisher_gui_node = launch_ros.actions.Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
    )
    rviz_node = launch_ros.actions.Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d",LaunchConfiguration("rvizconfig")],
    )

    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(name="model",default_value=default_model_path,description="Absolute path to robot urdf file"),
        launch.actions.DeclareLaunchArgument(name="rvizconfig",default_value=default_rviz_config_path,description="Absolute path to rviz config file"),
        join_state_publisher_gui_node,
        robot_state_publisher_node,
        rviz_node
    ])
