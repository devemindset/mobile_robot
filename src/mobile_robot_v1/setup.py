from setuptools import find_packages, setup

package_name = 'mobile_robot_v1'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ("share/" + package_name + "/launch",["launch/display.launch.py"]),
        ("share/" + package_name + "/urdf",["urdf/robot_mobile.urdf.xacro"]),
        ("share/" + package_name + "/urdf",["urdf/robot_core.xacro"]),
        ("share/" + package_name + "/urdf",["urdf/gazebo_control.xacro"]),
        ("share/" + package_name + "/rviz",["rviz/conf_chassis.rviz"]),
        ("share/" + package_name + "/launch",["launch/gazebo.launch.py"]),
        ("share/" + package_name + "/launch",["launch/combined_launch.py"]),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mona_robot',
    maintainer_email='mona_robot@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'keyboard_teleop = mobile_robot_v1.keyboard_teleop:main',
            'tf_broadcaster = mobile_robot_v1.tf_broadcaster:main'
        ],
    },
)
