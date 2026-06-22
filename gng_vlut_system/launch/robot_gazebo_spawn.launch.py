import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def launch_setup(context, *args, **kwargs):
    robot_name = LaunchConfiguration("robot_name").perform(context)
    robot_desc_pkg = get_package_share_directory(f"{robot_name}_description")
    robot_desc_default = os.path.join(robot_desc_pkg, "urdf", f"{robot_name}.urdf.xacro")
    if not os.path.exists(robot_desc_default):
        robot_desc_default = os.path.join(robot_desc_pkg, "urdf", f"{robot_name}_pro_normal.urdf.xacro")
    if not os.path.exists(robot_desc_default):
        raise FileNotFoundError(
            f"No URDF/Xacro found for robot_name='{robot_name}'. "
            f"Checked: {os.path.join(robot_desc_pkg, 'urdf', f'{robot_name}.urdf.xacro')}, "
            f"{os.path.join(robot_desc_pkg, 'urdf', f'{robot_name}_pro_normal.urdf.xacro')}"
        )

    robot_description_topic = f"/{robot_name}/robot_description"

    return [
        # Spawn Entity in Gazebo
        Node(
            package="gazebo_ros",
            executable="spawn_entity.py",
            arguments=[
                "-topic", robot_description_topic,
                "-entity", robot_name,
                "-x", "0", "-y", "0", "-z", "0"
            ],
            output="screen",
        )
    ]

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("robot_name", default_value="ToPoDualArm"),
        # We assume Gazebo is already running or started separately
        OpaqueFunction(function=launch_setup)
    ])
