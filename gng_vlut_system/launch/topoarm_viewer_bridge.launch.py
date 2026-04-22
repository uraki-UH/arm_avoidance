from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory("gng_safety")
    robot_description_file_default = package_share + "/temp_robot.urdf"
    resource_root_dir_default = package_share + "/urdf/topoarm_description"
    robot_mesh_root_dir_default = resource_root_dir_default + "/meshes/topoarm"

    robot_description_file = DeclareLaunchArgument(
        "robot_description_file",
        default_value=robot_description_file_default,
        description="URDF/Xacro file used for the robot arm bridge",
    )
    udp_port = DeclareLaunchArgument(
        "udp_port",
        default_value="12345",
        description="UDP port that feeds the topoarm joint state player",
    )
    joint_state_source = DeclareLaunchArgument(
        "joint_state_source",
        default_value="rviz",
        description="Which source should drive /joint_states: rviz or real",
    )
    resource_root_dir = DeclareLaunchArgument(
        "resource_root_dir",
        default_value=resource_root_dir_default,
        description="Root directory that contains urdf/ and meshes/",
    )
    mesh_root_dir = DeclareLaunchArgument(
        "mesh_root_dir",
        default_value=robot_mesh_root_dir_default,
        description="Mesh directory under the resource root",
    )
    end_effector_name = DeclareLaunchArgument(
        "end_effector_name",
        default_value="",
        description="Optional end-effector link name for the kinematic chain",
    )
    joint_state_topic = DeclareLaunchArgument(
        "joint_state_topic",
        default_value="/joint_states",
        description="JointState topic driving the robot arm pose",
    )
    stream_topic = DeclareLaunchArgument(
        "stream_topic",
        default_value="/viewer/internal/stream/robot_arm",
        description="Viewer bridge stream topic",
    )
    frame_id = DeclareLaunchArgument(
        "frame_id",
        default_value="world",
        description="Frame id used in the streamed robot arm payload",
    )
    publish_hz = DeclareLaunchArgument(
        "publish_hz",
        default_value="20.0",
        description="Robot arm publish rate",
    )
    runtime = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            get_package_share_directory("gng_safety") + "/launch/topoarm_runtime.launch.py"
        ),
        launch_arguments={
            "joint_state_source": LaunchConfiguration("joint_state_source"),
            "udp_port": LaunchConfiguration("udp_port"),
            "frame_id": LaunchConfiguration("frame_id"),
            "enable_safety_monitor": "false",
        }.items(),
    )

    bridge_node = Node(
        package="gng_safety",
        executable="robot_arm_bridge_node",
        name="robot_arm_bridge_node",
        output="screen",
        parameters=[{
            "robot_description_file": LaunchConfiguration("robot_description_file"),
            "resource_root_dir": LaunchConfiguration("resource_root_dir"),
            "mesh_root_dir": LaunchConfiguration("mesh_root_dir"),
            "end_effector_name": LaunchConfiguration("end_effector_name"),
            "joint_state_topic": LaunchConfiguration("joint_state_topic"),
            "stream_topic": LaunchConfiguration("stream_topic"),
            "frame_id": LaunchConfiguration("frame_id"),
            "publish_hz": LaunchConfiguration("publish_hz"),
        }],
    )

    return LaunchDescription([
        robot_description_file,
        udp_port,
        joint_state_source,
        resource_root_dir,
        mesh_root_dir,
        end_effector_name,
        joint_state_topic,
        stream_topic,
        frame_id,
        publish_hz,
        runtime,
        bridge_node,
    ])
