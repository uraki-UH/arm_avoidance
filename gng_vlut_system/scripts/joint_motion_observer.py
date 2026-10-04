#!/usr/bin/env python3
"""標準JointStateからの任意起動の運動状態観測。制御指令の出力なし。"""
import math

from control_msgs.msg import DynamicJointState, InterfaceValue
import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState

from joint_motion_state import joint_motion_tracker, motion_fields


class joint_motion_observer(Node):
    def __init__(self, **kwargs):
        super().__init__('joint_motion_observer', **kwargs)
        defaults = {'state_topic': 'joint_states', 'motion_state_topic': 'joint_motion_states',
                    'max_derivative_order': 3, 'min_sample_period_sec': 0.0001,
                    'max_sample_gap_sec': 0.5}
        config = {name: self.declare_parameter(name, value, ParameterDescriptor(read_only=True)).value
                  for name, value in defaults.items()}
        continuous_joint_names = self.declare_parameter(
            'continuous_joint_names', rclpy.Parameter.Type.STRING_ARRAY,
            ParameterDescriptor(read_only=True)).value or []
        self.tracker = joint_motion_tracker(
            config['max_derivative_order'], config['min_sample_period_sec'],
            config['max_sample_gap_sec'], continuous_joint_names)
        self.frame_id = None
        self.state_pub = self.create_publisher(DynamicJointState, config['motion_state_topic'], 10)
        self.state_sub = self.create_subscription(
            JointState, config['state_topic'], self.on_state, qos_profile_sensor_data)

    def on_state(self, message):
        if self.frame_id != message.header.frame_id:
            self.tracker.reset()
            self.frame_id = message.header.frame_id
        try:
            states = self.tracker.update(
                message.header.stamp.sec + message.header.stamp.nanosec * 1e-9,
                message.name, message.position, message.velocity)
        except ValueError as error:
            self.get_logger().warning(str(error), throttle_duration_sec=5.0)
            return
        if not states:
            return
        output = DynamicJointState()
        output.header = message.header
        output.joint_names = list(states)
        for state in states.values():
            interfaces = InterfaceValue()
            interfaces.interface_names = [*motion_fields, *['is_' + name + '_estimated' for name in motion_fields[1:]]]
            interfaces.values = [getattr(state, name) if getattr(state, name) is not None else math.nan
                                 for name in motion_fields]
            interfaces.values.extend(float(name in state.estimated_fields) for name in motion_fields[1:])
            output.interface_values.append(interfaces)
        self.state_pub.publish(output)


def main():
    rclpy.init()
    node = None
    try:
        node = joint_motion_observer()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
