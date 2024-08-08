import rclpy
import rclpy.node
import rclpy.qos
import rclpy.action

import sensor_msgs.msg
import std_msgs.msg
import astra_controller_interfaces.msg
import astra_controller_interfaces.srv
import threading
import struct
import numpy as np

from .arm_controller import ArmController

def main(args=None):
    rclpy.init(args=args)

    node = rclpy.node.Node('arm_node')

    logger = node.get_logger()
    
    node.declare_parameter('device', '/dev/tty_puppet_right')

    device = node.get_parameter('device').value

    arm_controller = ArmController(device)

    joint_state_publisher = node.create_publisher(sensor_msgs.msg.JointState, "/joint_states", 10)
    def cb(position, velocity, effort, this_time):
        msg = sensor_msgs.msg.JointState()
        msg.header.stamp = node.get_clock().now().to_msg()
                
        msg.name = [
            "joint_r2", 
            "joint_r3", 
            "joint_r4", 
            "joint_r5", 
            "joint_r6", 
            "joint_r7l", 
            "joint_r7r" 
        ]
        msg.position = [ 
            float(position[0]), 
            float(position[1]), 
            float(position[2]), 
            float(position[3]), 
            float(position[4]), 
            -float(position[5]), 
            float(position[5]), 
        ]
        msg.velocity = [ 
            float(velocity[0]), 
            float(velocity[1]), 
            float(velocity[2]), 
            float(velocity[3]), 
            float(velocity[4]), 
            -float(velocity[5]), 
            float(velocity[5]), 
        ]
        msg.effort = [ 
            float(effort[0]), 
            float(effort[1]), 
            float(effort[2]), 
            float(effort[3]), 
            float(effort[4]), 
            -float(effort[5]), 
            float(effort[5]), 
        ]

        joint_state_publisher.publish(msg)
    arm_controller.state_cb = cb

    pong_publisher = node.create_publisher(std_msgs.msg.UInt16MultiArray, '/pong', 10)
    def cb(data):
        logger.info(f'pong: {data}')
        pong_publisher.publish(std_msgs.msg.UInt16MultiArray(
            data=data,
        ))
    arm_controller.pong_cb = cb
    
    last_cmd = arm_controller.get_pos()[0]
    logger.info(f"using initial state {last_cmd}")
    
    def cb(msg: astra_controller_interfaces.msg.JointGroupCommand):
        nonlocal last_cmd
        cmd = [*msg.cmd[:5], last_cmd[5]]
        arm_controller.set_pos(cmd)
        last_cmd = cmd
    node.create_subscription(astra_controller_interfaces.msg.JointGroupCommand, '/arm_joint_command', cb, rclpy.qos.qos_profile_sensor_data)
    
    def cb(msg: astra_controller_interfaces.msg.JointGroupCommand):
        nonlocal last_cmd
        cmd = [*last_cmd[:5], msg.cmd[0]]
        arm_controller.set_pos(cmd)
        last_cmd = cmd
    node.create_subscription(astra_controller_interfaces.msg.JointGroupCommand, '/gripper_joint_command', cb, rclpy.qos.qos_profile_sensor_data)
    
    def cb(msg: astra_controller_interfaces.msg.JointGroupCommand):
        cmd_data = [round(x) for x in msg.position]
        logger.info(f'torque cmd: {cmd_data}')
        arm_controller.write(struct.pack('>BBHHHHHHH', arm_controller.COMM_HEAD, arm_controller.COMM_TYPE_TORQUE, *cmd_data))
    node.create_subscription(astra_controller_interfaces.msg.JointGroupCommand, '/torque_command', cb, rclpy.qos.qos_profile_sensor_data)
    
    def cb(msg: std_msgs.msg.UInt16MultiArray):
        cmd_data = msg.data
        logger.info(f'ping cmd: {cmd_data}')
        arm_controller.write(struct.pack('>BBHHHHHHH', arm_controller.COMM_HEAD, arm_controller.COMM_TYPE_PING, *cmd_data))
    node.create_subscription(std_msgs.msg.UInt16MultiArray, '/ping', cb, rclpy.qos.qos_profile_sensor_data)

    def cb(
        request: astra_controller_interfaces.srv.ReadConfig.Request, 
        response: astra_controller_interfaces.srv.ReadConfig.Response
    ):
        logger.info(f'read_config: {request.addr}')

        e = threading.Event()
        d = None
        def config_cb(data):
            nonlocal d
            d = struct.unpack('>xxIixxxxxxxx', data)
            e.set()
            
        with arm_controller.config_cb_lock:
            arm_controller.config_cb = config_cb
            arm_controller.write(struct.pack('>BBIxxxxxxxxxxxx', arm_controller.COMM_HEAD, arm_controller.COMM_TYPE_CONFIG_READ, request.addr))
            # or use future https://stackoverflow.com/questions/43550756/setting-asyncio-futures-value-in-a-callback-from-different-thread

            e.wait() # TODO 可能会出现先set后wait的情况(如果时序不对的话)

            arm_controller.config_cb = None

        assert(request.addr == d[0])
        response.data = d[1]
        
        return response
    node.create_service(astra_controller_interfaces.srv.ReadConfig, '/read_config', cb)

    def cb(
        request: astra_controller_interfaces.srv.WriteConfig.Request, 
        response: astra_controller_interfaces.srv.WriteConfig.Response
    ):
        logger.info(f'write_config: {request.addr} {request.data}')

        e = threading.Event()
        d = None
        def config_cb(data):
            nonlocal d
            d = struct.unpack('>xxIixxxxxxxx', data)
            e.set()
            
        with arm_controller.config_cb_lock:
            arm_controller.config_cb = config_cb
            arm_controller.write(struct.pack('>BBIixxxxxxxx', arm_controller.COMM_HEAD, arm_controller.COMM_TYPE_CONFIG_WRITE, request.addr, request.data))

            e.wait()

            arm_controller.config_cb = None

        assert(request.addr == d[0])
        response.data = d[1]
        
        return response
    node.create_service(astra_controller_interfaces.srv.WriteConfig, '/write_config', cb)

    def cb(
        request: astra_controller_interfaces.srv.ReadConfigFloat.Request, 
        response: astra_controller_interfaces.srv.ReadConfigFloat.Response
    ):
        logger.info(f'read_config_float: {request.addr}')

        e = threading.Event()
        d = None
        def config_cb(data):
            nonlocal d
            d = struct.unpack('>xxIfxxxxxxxx', data)
            e.set()
            
        with arm_controller.config_cb_lock:
            arm_controller.config_cb = config_cb
            arm_controller.write(struct.pack('>BBIxxxxxxxxxxxx', arm_controller.COMM_HEAD, arm_controller.COMM_TYPE_CONFIG_READ, request.addr))

            e.wait() # TODO 可能会出现先set后wait的情况(如果时序不对的话)

            arm_controller.config_cb = None

        assert(request.addr == d[0])
        response.data = d[1]
        
        return response
    node.create_service(astra_controller_interfaces.srv.ReadConfigFloat, '/read_config_float', cb)

    def cb(
        request: astra_controller_interfaces.srv.WriteConfigFloat.Request, 
        response: astra_controller_interfaces.srv.WriteConfigFloat.Response
    ):
        logger.info(f'write_config_float: {request.addr} {request.data}')

        e = threading.Event()
        d = None
        def config_cb(data):
            nonlocal d
            d = struct.unpack('>xxIfxxxxxxxx', data)
            e.set()
            
        with arm_controller.config_cb_lock:
            arm_controller.config_cb = config_cb
            arm_controller.write(struct.pack('>BBIfxxxxxxxx', arm_controller.COMM_HEAD, arm_controller.COMM_TYPE_CONFIG_WRITE, request.addr, request.data))

            e.wait()

            arm_controller.config_cb = None

        assert(request.addr == d[0])
        response.data = d[1]
        
        return response
    node.create_service(astra_controller_interfaces.srv.WriteConfigFloat, '/write_config_float', cb)

    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
