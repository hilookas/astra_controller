import rclpy
import rclpy.node
import rclpy.qos

import sensor_msgs.msg
import astra_controller_interfaces.msg
import numpy as np
import threading
import time

import logging

logger = logging.getLogger(__name__)

np.set_printoptions(precision=4, suppress=True)

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.node.Node("dry_run_node")

    joint_state_publisher = node.create_publisher(sensor_msgs.msg.JointState, "joint_states", 10)

    node.declare_parameter('actively_send_joint_state', False)

    actively_send_joint_state = node.get_parameter('actively_send_joint_state').value

    if actively_send_joint_state:
        # actively send current joint_state
        joint_states = {
            "joint_r1": 0,
            "joint_r2": 0,
            "joint_r3": 0,
            "joint_r4": 0,
            "joint_r5": 0,
            "joint_r6": 0,
            "joint_r7l": 0,
            "joint_r7r": 0
        }
        def publish_joint_states():
            while True:
                msg = sensor_msgs.msg.JointState()
                msg.header.stamp = node.get_clock().now().to_msg()

                msg.name = list(joint_states.keys())
                msg.position = [float(x) for x in joint_states.values()]

                joint_state_publisher.publish(msg)
                time.sleep(0.1)
        t = threading.Thread(target=publish_joint_states, daemon=True)
        t.start()

    def cb(msg: astra_controller_interfaces.msg.JointGroupCommand):
        send_msg = sensor_msgs.msg.JointState()
        send_msg.header.stamp = node.get_clock().now().to_msg()

        send_msg.name = [ "joint_r2", "joint_r3", "joint_r4", "joint_r5", "joint_r6" ]
        send_msg.position = msg.cmd
        joint_state_publisher.publish(send_msg)

        if actively_send_joint_state:
            joint_states.update(dict(zip(send_msg.name, send_msg.position)))
    node.create_subscription(astra_controller_interfaces.msg.JointGroupCommand, "arm/joint_command", cb, rclpy.qos.qos_profile_sensor_data)

    def cb(msg: astra_controller_interfaces.msg.JointGroupCommand):
        send_msg = sensor_msgs.msg.JointState()
        send_msg.header.stamp = node.get_clock().now().to_msg()

        send_msg.name = [ "joint_r7r", "joint_r7l" ]
        send_msg.position = [ msg.cmd[0], -msg.cmd[0] ]
        joint_state_publisher.publish(send_msg)

        if actively_send_joint_state:
            joint_states.update(dict(zip(send_msg.name, send_msg.position)))
    node.create_subscription(astra_controller_interfaces.msg.JointGroupCommand, "arm/gripper_joint_command", cb, rclpy.qos.qos_profile_sensor_data)

    def cb(msg: astra_controller_interfaces.msg.JointGroupCommand):
        send_msg = sensor_msgs.msg.JointState()
        send_msg.header.stamp = node.get_clock().now().to_msg()

        send_msg.name = [ "joint_r1" ]
        send_msg.position = msg.cmd
        joint_state_publisher.publish(send_msg)

        if actively_send_joint_state:
            joint_states.update(dict(zip(send_msg.name, send_msg.position)))
    node.create_subscription(astra_controller_interfaces.msg.JointGroupCommand, "lift_joint_command", cb, rclpy.qos.qos_profile_sensor_data)

    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
