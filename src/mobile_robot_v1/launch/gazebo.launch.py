import launch
from launch.substitutions import Command, LaunchConfiguration
import launch_ros
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
import os 

def generate_launch_description():
    pkg_share = launch_ros.substitutions.FindPackageShare(package="mobile_robot_v1").find("mobile_robot_v1")
    default_model_path = os.path.join(pkg_share,"urdf/robot_mobile.urdf.xacro")
    
    use_sim_time = LaunchConfiguration("use_sim_time", default="true")

    
    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"use_sim_time" : use_sim_time,'robot_description' : Command(["xacro " , LaunchConfiguration("model")])}]
    )
 
    
    joint_state_publisher_node = launch_ros.actions.Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}]
    )
    spawn_entity = launch_ros.actions.Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-entity","robot_mobile","-topic","robot_description"],
        output = "screen"
    )

    return launch.LaunchDescription(
        [
            launch.actions.DeclareLaunchArgument(name="model",default_value=default_model_path,
                                                 description="Absolute path to robot urdf file"),
            launch.actions.ExecuteProcess(cmd=["gazebo","--verbose","-s","libgazebo_ros_init.so","-s","libgazebo_ros_factory.so"],output="screen"),
            joint_state_publisher_node,
            robot_state_publisher_node,
            spawn_entity,
            #gazebo_launch
        ]
    ) 