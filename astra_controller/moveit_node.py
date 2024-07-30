import rclpy
import rclpy.node
import rclpy.qos
import rclpy.action

import sensor_msgs.msg
import control_msgs.action
import serial
import threading
import numpy as np
import struct
import math
import time
import sys
import astra_controller_interfaces.srv

class ArmController:
    COMM_LEN = 2 + 16
    COMM_HEAD = 0x5A
    COMM_TYPE_PING = 0x00
    COMM_TYPE_PONG = 0x01
    COMM_TYPE_CTRL = 0x02
    COMM_TYPE_FEEDBACK = 0x03
    COMM_TYPE_TORQUE = 0x04
    COMM_TYPE_CONFIG_WRITE = 0x05
    COMM_TYPE_CONFIG_READ = 0x06
    COMM_TYPE_CONFIG_FEEDBACK = 0x07
        
    GRIPPER_GEAR_R = 0.027 / 2

    @staticmethod
    def to_si_unit(arr):
        arr = -(np.array(arr) - 2048) / 4096 * (2*math.pi)
        arr[-1] *= ArmController.GRIPPER_GEAR_R
        arr[-1] += 0.03 # 标定的时候，中点要设置为夹爪张开60mm的时候（左右各30mm）
        return arr

    @staticmethod
    def to_raw_unit(arr):
        arr[-1] -= 0.03
        arr[-1] /= ArmController.GRIPPER_GEAR_R
        arr = (-np.array(arr) / (2*math.pi) * 4096 + 2048).astype(int)
        return arr

    def __init__(self, name, state_cb=None, pong_cb=None):
        logger.info(name)

        self.state_cb = state_cb
        self.pong_cb = pong_cb

        self.ser = serial.Serial(name, 921600, timeout=None)
        # self.ser.rts = False

        self.lock = threading.Lock()
        self.last_time = None
        
        self.last_position = None
        self.last_velocity = None
        self.last_effort = None
        self.last_time = None

        self.write_lock = threading.Lock()

        self.config_cb = None
        self.config_cb_lock = threading.Lock()

        self.quit = threading.Event()

        self.t = threading.Thread(target=self.recv_thread)
        self.t.daemon = True
        self.t.start()

        while self.last_position is None: # wait for init done
            time.sleep(0.1)

    def recv_thread(self):
        try:
            while not self.quit.is_set():
                data = self.ser.read(1)
                if not (data[0] == self.COMM_HEAD): # 逐步同步
                    sys.stdout.buffer.write(data)
                    sys.stdout.flush()
                    continue
                
                data += self.ser.read(self.COMM_LEN - 1)
                assert(len(data) == self.COMM_LEN)

                if data[1] == self.COMM_TYPE_PONG:
                    if self.state_cb is not None:
                        self.pong_cb(struct.unpack('>xxHHHHHHxxxx', data))
                elif data[1] == self.COMM_TYPE_FEEDBACK:
                    position = self.to_si_unit(np.array(struct.unpack('>xxHHHHHHxxxx', data)))
                    this_time = time.time()
                    with self.lock:
                        if self.last_time is None:
                            self.last_position = position
                            self.last_velocity = np.array([0, 0, 0, 0, 0, 0])
                            self.last_effort = np.array([0, 0, 0, 0, 0, 0])
                            self.last_time = this_time - 1 # in case of dividing 0
                        delta_time = this_time - self.last_time
                        velocity = (position - self.last_position) / delta_time 
                        effort = (velocity - self.last_velocity) / delta_time # without bias (gravity) and mass
                        self.last_position = position
                        self.last_velocity = velocity
                        self.last_effort = effort
                        self.last_time = this_time
                        if self.state_cb is not None:
                            self.state_cb(self.last_position, self.last_velocity, self.last_effort, self.last_time)
                elif data[1] == self.COMM_TYPE_CONFIG_FEEDBACK:
                    self.config_cb(data) # TODO 解决没有时报错
                else:
                    sys.stdout.buffer.write(data)
                    sys.stdout.flush()
        # except Exception:
        #     pass
        finally:
            print("thread exiting")

    def get_pos(self): # 一帧可能会被用多次，但是绝对不会卡住
        with self.lock:
            return self.last_position, self.last_velocity, self.last_effort, self.last_time

    def write(self, encoded_data):
        with self.write_lock:
            # print(encoded_data)
            self.ser.write(encoded_data)

    JOINT_MIN = np.array([-math.pi/2, -math.pi/2, -math.pi*0.75, -math.pi/2, -math.pi*0.75, -math.pi/2])
    JOINT_MAX = np.array([math.pi/2, math.pi/2, math.pi*0.75, math.pi/2, math.pi*0.75, math.pi/2])

    def set_pos(self, pos):
        pos_protected = np.minimum(np.maximum(np.array(pos), self.JOINT_MIN), self.JOINT_MAX)
        if all(np.isclose(pos, pos_protected, atol=0.01)):
            self.write(struct.pack('>BBHHHHHHxxxx', self.COMM_HEAD, self.COMM_TYPE_CTRL, *self.to_raw_unit(pos)))
        else:
            logger.error(f"Arm reach min/max!!! {pos} {pos_protected}")
        
    def __del__(self):
        self.quit.set()
        self.ser.close()

class LiftController:
    COMM_LEN = 2 + 16
    COMM_HEAD = 0x5A
    COMM_TYPE_PING = 0x00
    COMM_TYPE_PONG = 0x01
    COMM_TYPE_CTRL = 0x02
    COMM_TYPE_FEEDBACK = 0x03

    STEPPER_PULSE_PER_REV = 800 # 800pulse/rev
    RAIL_MM_PER_REV = 10 # 10mm/rev
    RAIL_MAX_LENGTH_MM = 500 # 1000mm length rail // 500mm on desk length
    STEPPER_MAX_PULSE = (RAIL_MAX_LENGTH_MM / RAIL_MM_PER_REV * STEPPER_PULSE_PER_REV) # 80000

    @staticmethod
    def to_si_unit(x):
        return x / LiftController.STEPPER_PULSE_PER_REV * LiftController.RAIL_MM_PER_REV / 1000
        
    @staticmethod
    def to_raw_unit(x):
        return int(x * 1000 / LiftController.RAIL_MM_PER_REV * LiftController.STEPPER_PULSE_PER_REV)

    def __init__(self, name, state_cb=None, pong_cb=None):
        logger.info(name)

        self.state_cb = state_cb
        self.pong_cb = pong_cb

        self.ser = serial.Serial(name, 921600, timeout=None)
        # self.ser.rts = False

        self.lock = threading.Lock()
        self.last_time = None
        
        self.last_position = None
        self.last_velocity = None
        self.last_effort = None
        self.last_time = None

        self.write_lock = threading.Lock()

        self.quit = threading.Event()

        self.t = threading.Thread(target=self.recv_thread)
        self.t.daemon = True
        self.t.start()

        while self.last_position is None: # wait for init done
            time.sleep(0.1)

    def recv_thread(self):
        try:
            while not self.quit.is_set():
                data = self.ser.read(1)
                if not (data[0] == self.COMM_HEAD): # 逐步同步
                    sys.stdout.buffer.write(data)
                    sys.stdout.flush()
                    continue
                
                data += self.ser.read(self.COMM_LEN - 1)
                assert(len(data) == self.COMM_LEN)

                if data[1] == self.COMM_TYPE_PONG:
                    if self.state_cb is not None:
                        self.pong_cb(struct.unpack('>xxIxxxxxxxxxxxx', data))
                elif data[1] == self.COMM_TYPE_FEEDBACK:
                    position = self.to_si_unit(np.array(struct.unpack('>xxIxxxxxxxxxxxx', data)))
                    this_time = time.time()
                    with self.lock:
                        if self.last_time is None:
                            self.last_position = 0
                            self.last_velocity = 0
                            self.last_effort = 0
                            self.last_time = this_time - 1 # in case of dividing 0
                        delta_time = this_time - self.last_time
                        velocity = (position - self.last_position) / delta_time 
                        effort = (velocity - self.last_velocity) / delta_time # without bias (gravity) and mass
                        self.last_position = position
                        self.last_velocity = velocity
                        self.last_effort = effort
                        self.last_time = this_time
                        if self.state_cb is not None:
                            self.state_cb(self.last_position, self.last_velocity, self.last_effort, self.last_time)
                else:
                    sys.stdout.buffer.write(data)
                    sys.stdout.flush()
        # except Exception:
        #     pass
        finally:
            print("thread exiting")

    def get_pos(self): # 一帧可能会被用多次，但是绝对不会卡住
        with self.lock:
            return self.last_position, self.last_velocity, self.last_effort, self.last_time

    def write(self, encoded_data):
        with self.write_lock:
            # print(encoded_data)
            self.ser.write(encoded_data)

    def set_pos(self, pos):
        pos_protected = np.minimum(np.maximum(pos, 0), (self.RAIL_MAX_LENGTH_MM / 1000))
        if np.isclose(pos, pos_protected, atol=0.01):
            self.write(struct.pack('>BBIxxxxxxxxxxxx', self.COMM_HEAD, self.COMM_TYPE_CTRL, self.to_raw_unit(pos)))
        else:
            logger.error(f"Arm reach min/max!!! {pos} {pos_protected}")
        
    def __del__(self):
        self.quit.set()
        self.ser.close()

def main(args=None):
    rclpy.init(args=args)

    node = rclpy.node.Node('my_node')

    global logger
    logger = node.get_logger()
    
    # node.declare_parameter('device', '/dev/tty_puppet_right')

    # device = node.get_parameter('device').value

    right_controller = ArmController("/dev/tty_puppet_right")
    
    right_lift_controller = LiftController("/dev/tty_puppet_lift_right")

    joint_state_publisher = node.create_publisher(sensor_msgs.msg.JointState, "/joint_states", 10)

    joint_pos = {
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
            
            msg.name = [
                "joint_r1", 
                "joint_r2", 
                "joint_r3", 
                "joint_r4", 
                "joint_r5", 
                "joint_r6", 
                "joint_r7l", 
                "joint_r7r" 
            ]
            msg.position = [ 
                float(right_lift_controller.last_position), 
                float(right_controller.last_position[0]), 
                float(right_controller.last_position[1]), 
                float(right_controller.last_position[2]), 
                float(right_controller.last_position[3]), 
                float(right_controller.last_position[4]), 
                -float(right_controller.last_position[5]), 
                float(right_controller.last_position[5])
            ]

            joint_state_publisher.publish(msg)
            time.sleep(0.1)
    t = threading.Thread(target=publish_joint_states)
    t.daemon = True
    t.start()

    # def state_cb(position, velocity, effort, this_time):
    #     msg = sensor_msgs.msg.JointState()
    #     msg.header.stamp = node.get_clock().now().to_msg()
        
    #     msg.name = [
    #         "joint_r1", 
    #         "joint_r2", 
    #         "joint_r3", 
    #         "joint_r4", 
    #         "joint_r5", 
    #         "joint_r6", 
    #         "joint_r7l", 
    #         "joint_r7r" 
    #     ]
    #     msg.position = [ 
    #         float(0.1), 
    #         float(position[0]), 
    #         float(position[1]), 
    #         float(position[2]), 
    #         float(position[3]), 
    #         float(position[4]), 
    #         -float(position[5]), 
    #         float(position[5]), 
    #     ]

    #     joint_state_publisher.publish(msg)
    # right_controller.state_cb = state_cb

    def execute_callback(goal_handle: rclpy.action.server.ServerGoalHandle):
        # https://docs.ros.org/en/foxy/Tutorials/Intermediate/Writing-an-Action-Server-Client/Py.html
        node.get_logger().info('Executing goal...')
        # print(goal_handle.request.trajectory.joint_names) # ['joint_r1', 'joint_r2', 'joint_r3', 'joint_r4', 'joint_r5', 'joint_r6']
        # print(goal_handle.request.trajectory.points[-1].positions) # trajectory_msgs.msg.JointTrajectoryPoint(positions=[0.1,0.2,...], velocities=...)

        for joint_name, position in zip(
            goal_handle.request.trajectory.joint_names, 
            goal_handle.request.trajectory.points[-1].positions
        ):
            joint_pos[joint_name] = position
        
        # ignore publish_feedback

        goal_handle.succeed()

        result = control_msgs.action.FollowJointTrajectory.Result()
        result.error_code = control_msgs.action.FollowJointTrajectory.Result.SUCCESSFUL
        result.error_string = "SUCC"
        return result
    action_server = rclpy.action.ActionServer(
        node,
        control_msgs.action.FollowJointTrajectory,
        '/astra_right_arm_controller/follow_joint_trajectory',
        execute_callback
    )

    def execute_callback(goal_handle: rclpy.action.server.ServerGoalHandle):
        # https://docs.ros.org/en/foxy/Tutorials/Intermediate/Writing-an-Action-Server-Client/Py.html
        node.get_logger().info('Executing goal...')
        # print(goal_handle.request.trajectory.joint_names) # ['joint_r1', 'joint_r2', 'joint_r3', 'joint_r4', 'joint_r5', 'joint_r6']
        # print(goal_handle.request.trajectory.points[-1].positions) # trajectory_msgs.msg.JointTrajectoryPoint(positions=[0.1,0.2,...], velocities=...)

        for joint_name, position in zip(
            goal_handle.request.trajectory.joint_names, 
            goal_handle.request.trajectory.points[-1].positions
        ):
            joint_pos[joint_name] = position
        
        # ignore publish_feedback

        goal_handle.succeed()

        result = control_msgs.action.FollowJointTrajectory.Result()
        result.error_code = control_msgs.action.FollowJointTrajectory.Result.SUCCESSFUL
        result.error_string = "SUCC"
        return result
    action_server = rclpy.action.ActionServer(
        node,
        control_msgs.action.FollowJointTrajectory,
        '/astra_right_hand_controller/follow_joint_trajectory',
        execute_callback
    )
    
    def do_pose():
        while True:
            right_controller.set_pos([joint_pos["joint_r2"], joint_pos["joint_r3"], joint_pos["joint_r4"], joint_pos["joint_r5"], joint_pos["joint_r6"], joint_pos["joint_r7r"]])
            right_lift_controller.set_pos(joint_pos["joint_r1"])
            time.sleep(0.01)
    t = threading.Thread(target=do_pose)
    t.daemon = True
    t.start()

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
            
        with right_controller.config_cb_lock:
            right_controller.config_cb = config_cb
            right_controller.write(struct.pack('>BBIxxxxxxxxxxxx', right_controller.COMM_HEAD, right_controller.COMM_TYPE_CONFIG_READ, request.addr))
            # or use future https://stackoverflow.com/questions/43550756/setting-asyncio-futures-value-in-a-callback-from-different-thread

            e.wait() # TODO 可能会出现先set后wait的情况(如果时序不对的话)

            right_controller.config_cb = None

        assert(request.addr == d[0])
        response.data = d[1]
        
        return response
    node.create_service(astra_controller_interfaces.srv.ReadConfig, f'/read_config', cb)

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
            
        with right_controller.config_cb_lock:
            right_controller.config_cb = config_cb
            right_controller.write(struct.pack('>BBIixxxxxxxx', right_controller.COMM_HEAD, right_controller.COMM_TYPE_CONFIG_WRITE, request.addr, request.data))

            e.wait()

            right_controller.config_cb = None

        assert(request.addr == d[0])
        response.data = d[1]
        
        return response
    node.create_service(astra_controller_interfaces.srv.WriteConfig, f'/write_config', cb)

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
            
        with right_controller.config_cb_lock:
            right_controller.config_cb = config_cb
            right_controller.write(struct.pack('>BBIxxxxxxxxxxxx', right_controller.COMM_HEAD, right_controller.COMM_TYPE_CONFIG_READ, request.addr))

            e.wait() # TODO 可能会出现先set后wait的情况(如果时序不对的话)

            right_controller.config_cb = None

        assert(request.addr == d[0])
        response.data = d[1]
        
        return response
    node.create_service(astra_controller_interfaces.srv.ReadConfigFloat, f'/read_config_float', cb)

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
            
        with right_controller.config_cb_lock:
            right_controller.config_cb = config_cb
            right_controller.write(struct.pack('>BBIfxxxxxxxx', right_controller.COMM_HEAD, right_controller.COMM_TYPE_CONFIG_WRITE, request.addr, request.data))

            e.wait()

            right_controller.config_cb = None

        assert(request.addr == d[0])
        response.data = d[1]
        
        return response
    node.create_service(astra_controller_interfaces.srv.WriteConfigFloat, f'/write_config_float', cb)

    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
