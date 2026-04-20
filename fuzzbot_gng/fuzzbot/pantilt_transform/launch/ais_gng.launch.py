from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler, OpaqueFunction, ExecuteProcess
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.descriptions import ParameterValue

def generate_launch_description():
   realsense_path = PythonLaunchDescriptionSource([PathJoinSubstitution([FindPackageShare("realsense2_camera"),"launch","rs_launch.py"])])
   
   realsense_launch = IncludeLaunchDescription(realsense_path, 
                    launch_arguments = {
                        'pointcloud.enable': 'True',
                        'rgb_camera.color_profile': '424x240x15',
                        'depth_module.depth_profile': '424x240x15',
                        'enable_decimation_filter': 'True',
                        'spatial_filter.enable': 'True',
                        'temporal_filter.enable': 'True',
                        'disparity_filter.enable': 'False',
                        'depth_module.hdr_enabled': 'False',
                        'clip_distance': '5.0'
                    }.items())
   
   fuzzbot_hardware_node = Node(
            package="fuzzbot_hardware",
            executable="fuzzbot_hardware")

   transform_node = Node(
            package="pantilt_transform",
            executable="transform",
            parameters=[{'voxel': 0.05}])
   
   ais_gng_node = Node(
            package="ais_gng",
            executable="ais_gng",
            parameters=[{'filter.z_min': -10.0},
                        {'filter.z_max': 10.0}])

   return LaunchDescription([
        realsense_launch,
        fuzzbot_hardware_node,
        transform_node,
        ais_gng_node
    ])