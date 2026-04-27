from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Declare launch arguments for static transform
    x_pos_arg = DeclareLaunchArgument(
        'x_pos', default_value='0.0',
        description='X position of the camera relative to the arm base link'
    )
    y_pos_arg = DeclareLaunchArgument(
        'y_pos', default_value='0.0',
        description='Y position of the camera relative to the arm base link'
    )
    z_pos_arg = DeclareLaunchArgument(
        'z_pos', default_value='0.0',
        description='Z position of the camera relative to the arm base link'
    )
    roll_arg = DeclareLaunchArgument(
        'roll', default_value='0.0',
        description='Roll orientation of the camera relative to the arm base link (radians)'
    )
    pitch_arg = DeclareLaunchArgument(
        'pitch', default_value='0.0',
        description='Pitch orientation of the camera relative to the arm base link (radians)'
    )
    yaw_arg = DeclareLaunchArgument(
        'yaw', default_value='0.0',
        description='Yaw orientation of the camera relative to the arm base link (radians)'
    )
    parent_frame_arg = DeclareLaunchArgument(
        'parent_frame', default_value='arm_base_link',
        description='Parent frame ID for the static transform'
    )
    child_frame_arg = DeclareLaunchArgument(
        'child_frame', default_value='camera_link',
        description='Child frame ID for the static transform (camera frame)'
    )
    target_frame_arg = DeclareLaunchArgument(
        'target_frame', default_value='arm_base_link',
        description='Target frame ID for the point cloud transformation'
    )

    return LaunchDescription([
        x_pos_arg,
        y_pos_arg,
        z_pos_arg,
        roll_arg,
        pitch_arg,
        yaw_arg,
        parent_frame_arg,
        child_frame_arg,
        target_frame_arg,

        # Static Transform Publisher Node
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='arm_base_to_camera_tf_publisher',
            arguments=[
                LaunchConfiguration('x_pos'),
                LaunchConfiguration('y_pos'),
                LaunchConfiguration('z_pos'),
                LaunchConfiguration('roll'),
                LaunchConfiguration('pitch'),
                LaunchConfiguration('yaw'),
                LaunchConfiguration('parent_frame'),
                LaunchConfiguration('child_frame')
            ],
            output='screen'
        ),

        # PointCloud Transformer Node
        Node(
            package='pointcloud_transformer',
            executable='transformer_node',
            name='pointcloud_transformer_node',
            parameters=[{
                'target_frame': LaunchConfiguration('target_frame')
            }],
            output='screen',
            remappings=[
                ('/camera/depth/color/points', '/camera/depth/color/points'), # Input topic
                ('/transformed_points', '/transformed_points') # Output topic
            ]
        )
    ])
