from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():    
    node_joy = Node(
        package="joy",
        executable="joy_node",
        output="both"
    )
    
    node_fuzzbot_joycontroller = Node(
        package="fuzzbot_joycontroller",
        executable="fuzzbot_joycontroller",
        output="both"
    )
    
    nodes = [
        node_joy,
        node_fuzzbot_joycontroller
    ]
    
    return LaunchDescription(nodes)