import rclpy
import rclpy.node
import rclpy.qos
import rclpy.action

import sensor_msgs.msg
import astra_controller_interfaces.msg
import astra_controller_interfaces.srv
import std_msgs.msg

from .lift_controller import LiftController

def main(args=None):
    rclpy.init(args=args)

    node = rclpy.node.Node('lift_node')

    logger = node.get_logger()
    
    node.declare_parameter('device', '/dev/tty_puppet_lift_right')
    node.declare_parameter('joint_names', [ "joint_r1" ])

    device = node.get_parameter('device').value
    joint_names = node.get_parameter('joint_names').value
    
    assert len(joint_names) == 1

    lift_controller = LiftController(device)

    joint_state_publisher = node.create_publisher(sensor_msgs.msg.JointState, "joint_states", 10)
    def cb(position, velocity, effort, this_time):
        msg = sensor_msgs.msg.JointState()
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.name = joint_names
        msg.position = [ float(position) ]
        msg.velocity = [ float(velocity) ]
        msg.effort = [ float(effort) ]
        joint_state_publisher.publish(msg)
    lift_controller.state_cb = cb

    error_publisher = node.create_publisher(std_msgs.msg.String, 'error', 10)
    def cb(data):
        error_publisher.publish(std_msgs.msg.String(data=data))
    lift_controller.error_cb = cb
    
    def cb(msg: astra_controller_interfaces.msg.JointCommand):
        assert msg.name == joint_names
        lift_controller.set_pos(msg.position_cmd[0])
    node.create_subscription(astra_controller_interfaces.msg.JointCommand, 'joint_command', cb, rclpy.qos.qos_profile_sensor_data)

    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
