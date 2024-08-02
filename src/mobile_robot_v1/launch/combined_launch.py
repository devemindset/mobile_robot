import launch
from launch.substitutions import Command, LaunchConfiguration
import launch_ros
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = launch_ros.substitutions.FindPackageShare(package="mobile_robot_v1").find("mobile_robot_v1")
    default_model_path = os.path.join(pkg_share, "urdf/robot_mobile.urdf.xacro")
    default_rviz_config_path = os.path.join(pkg_share, "rviz/conf_chassis.rviz")
    use_sim_time = LaunchConfiguration("use_sim_time", default="true")

    robot_state_publisher_node = launch_ros.actions.Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"use_sim_time": use_sim_time, "robot_description": Command(["xacro ", LaunchConfiguration("model")])}]
    )

    joint_state_publisher_node = launch_ros.actions.Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}]
    )

    joint_state_publisher_gui_node = launch_ros.actions.Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
    )

    rviz_node = launch_ros.actions.Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", LaunchConfiguration("rvizconfig")],
    )

    spawn_entity = launch_ros.actions.Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-entity", "robot_mobile", "-topic", "robot_description"],
        output="screen"
    )

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(get_package_share_directory("gazebo_ros"), "launch", "gazebo.launch.py")]),
        launch_arguments={"use_sim_time": use_sim_time}.items()
    )

    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(name="model", default_value=default_model_path, description="Absolute path to robot urdf file"),
        launch.actions.DeclareLaunchArgument(name="rvizconfig", default_value=default_rviz_config_path, description="Absolute path to rviz config file"),
        launch.actions.ExecuteProcess(cmd=["gazebo", "--verbose", "-s", "libgazebo_ros_init.so", "-s", "libgazebo_ros_factory.so"], output="screen"),
        joint_state_publisher_node,
        joint_state_publisher_gui_node,
        robot_state_publisher_node,
        rviz_node,
        gazebo_launch,
        spawn_entity,
    ])
