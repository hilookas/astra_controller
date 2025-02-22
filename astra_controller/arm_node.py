import rclpy
import rclpy.node
import rclpy.qos
import rclpy.action

import sensor_msgs.msg
import std_msgs.msg
import astra_controller_interfaces.msg
import struct

from .arm_controller import ArmController

def main(args=None):
    rclpy.init(args=args)

    node = rclpy.node.Node('arm_node')

    logger = node.get_logger()
    
    node.declare_parameter('device', '/dev/tty_puppet_right')
    node.declare_parameter('joint_names', [ "joint_r2", "joint_r3", "joint_r4", "joint_r5", "joint_r6" ])
    node.declare_parameter('gripper_joint_names', [ "joint_r7l", "joint_r7r" ])

    device = node.get_parameter('device').value
    joint_names = node.get_parameter('joint_names').value
    gripper_joint_names = node.get_parameter('gripper_joint_names').value
    
    assert len(joint_names) == 5
    assert len(gripper_joint_names) == 2

    arm_controller = ArmController(device)
    
    joint_state_publisher = node.create_publisher(sensor_msgs.msg.JointState, "joint_states", 10)
    gripper_joint_state_publisher = node.create_publisher(sensor_msgs.msg.JointState, "gripper_joint_states", 10)
    def cb(position, velocity, effort, this_time):
        msg = sensor_msgs.msg.JointState()
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.name = joint_names
        msg.position = [ float(p) for p in position[:5] ]
        msg.velocity = [ float(v) for v in velocity[:5] ]
        msg.effort = [ float(e) for e in effort[:5] ]
        joint_state_publisher.publish(msg)
        
        msg = sensor_msgs.msg.JointState()
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.name = gripper_joint_names
        msg.position = [ -float(position[5]), float(position[5]) ]
        msg.velocity = [ -float(velocity[5]), float(velocity[5]) ]
        msg.effort = [ -float(effort[5]), float(effort[5]) ]
        gripper_joint_state_publisher.publish(msg)
    arm_controller.state_cb = cb

    debug_publisher = node.create_publisher(std_msgs.msg.Float32MultiArray, 'debug', 10)
    def cb(data):
        msg = std_msgs.msg.Float32MultiArray()
        msg.data = data
        debug_publisher.publish(msg)
    arm_controller.debug_cb = cb

    pong_publisher = node.create_publisher(std_msgs.msg.UInt16MultiArray, 'pong', 10)
    def cb(data):
        logger.info(f'pong: {data}')
        pong_publisher.publish(std_msgs.msg.UInt16MultiArray(
            data=data,
        ))
    arm_controller.pong_cb = cb

    error_publisher = node.create_publisher(std_msgs.msg.String, 'error', 10)
    def cb(data):
        error_publisher.publish(std_msgs.msg.String(data=data))
    arm_controller.error_cb = cb
    
    last_position_cmd = arm_controller.get_pos()[0]
    logger.info(f"using initial state {last_position_cmd}")
    
    def cb(msg: astra_controller_interfaces.msg.JointCommand):
        nonlocal last_position_cmd
        assert msg.name == joint_names
        position_cmd = [*msg.position_cmd[:5], last_position_cmd[5]]
        arm_controller.set_pos(position_cmd)
        last_position_cmd = position_cmd
    node.create_subscription(astra_controller_interfaces.msg.JointCommand, 'joint_command', cb, rclpy.qos.qos_profile_sensor_data)
    
    def cb(msg: astra_controller_interfaces.msg.JointCommand):
        nonlocal last_position_cmd
        assert msg.name == gripper_joint_names[1:2]
        position_cmd = [*last_position_cmd[:5], msg.position_cmd[0]]
        arm_controller.set_pos(position_cmd)
        last_position_cmd = position_cmd
    node.create_subscription(astra_controller_interfaces.msg.JointCommand, 'gripper_joint_command', cb, rclpy.qos.qos_profile_sensor_data)
    
    def cb(msg: std_msgs.msg.UInt8):
        logger.info(f'torque_enable: {msg.data}')
        arm_controller.set_torque(msg.data)
    node.create_subscription(std_msgs.msg.UInt8, 'torque_enable', cb, rclpy.qos.qos_profile_sensor_data)
    
    def cb(msg: std_msgs.msg.UInt16MultiArray):
        logger.info(f'ping: {msg.data}')
        arm_controller.write(struct.pack('>BBHHHHHHHH', arm_controller.COMM_HEAD, arm_controller.COMM_TYPE_PING, *msg.data))
    node.create_subscription(std_msgs.msg.UInt16MultiArray, 'ping', cb, rclpy.qos.qos_profile_sensor_data)
    
    def cb(msg: std_msgs.msg.Float32MultiArray):
        logger.info(f'set_pid: {msg.data}')
        arm_controller.set_pid(*msg.data)
    node.create_subscription(std_msgs.msg.Float32MultiArray, 'set_pid', cb, rclpy.qos.qos_profile_sensor_data)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt as err:
        arm_controller.stop()
        raise err

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
