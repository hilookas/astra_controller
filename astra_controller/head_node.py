import rclpy
import rclpy.node
import rclpy.publisher

import rclpy
import rclpy.node
import rclpy.qos

import astra_controller_interfaces.msg
import sensor_msgs.msg
import std_msgs.msg

from astra_controller.head_controller import HeadController

import numpy as np

def main(args=None):
    rclpy.init(args=args)

    node = rclpy.node.Node("head_node")

    logger = node.get_logger()
    
    node.declare_parameter('device', '/dev/tty_head')

    device = node.get_parameter('device').value
    joint_names = [ "joint_head_pan", "joint_head_tilt" ]

    head_controller = HeadController(device)

    joint_state_publisher = node.create_publisher(sensor_msgs.msg.JointState, "joint_states", 10)
    def cb(position, velocity, effort, this_time):
        msg = sensor_msgs.msg.JointState()
        msg.header.stamp = node.get_clock().now().to_msg()
        assert len(position) == len(velocity) == len(effort) == len(joint_names)
        msg.name = joint_names
        msg.position = [ float(p) for p in position ]
        msg.velocity = [ float(v) for v in velocity ]
        msg.effort = [ float(e) for e in effort ]
        joint_state_publisher.publish(msg)
    head_controller.state_cb = cb
    
    def cb(msg: astra_controller_interfaces.msg.JointCommand):
        assert msg.name == joint_names
        assert len(msg.position_cmd) == len(joint_names)
        position_cmd = msg.position_cmd
        head_controller.set_pos(np.array(position_cmd, dtype=np.float32))
    node.create_subscription(astra_controller_interfaces.msg.JointCommand, 'joint_command', cb, rclpy.qos.qos_profile_sensor_data)
    
    def cb(msg: std_msgs.msg.UInt8):
        logger.info(f'torque_enable: {msg.data}')
        head_controller.set_torque(msg.data)
    node.create_subscription(std_msgs.msg.UInt8, 'torque_enable', cb, rclpy.qos.qos_profile_sensor_data)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt as err:
        head_controller.stop()
        raise err
    
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()