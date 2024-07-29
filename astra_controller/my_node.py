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

class ArmController:
    COMM_HEAD = 0x5A
    COMM_TYPE_PING = 0x00
    COMM_TYPE_PONG = 0x01
    COMM_TYPE_CTRL = 0x02
    COMM_TYPE_FEEDBACK = 0x03
    COMM_TYPE_TORQUE = 0x04
    COMM_TYPE_CONFIG_WRITE = 0x05
    COMM_TYPE_CONFIG_READ = 0x06
    COMM_TYPE_CONFIG_FEEDBACK = 0x07

    @staticmethod
    def to_si_unit(arr):
        return (np.array(arr) - 2048) / 4096 * (2*math.pi)
        
    @staticmethod
    def to_raw_unit(arr):
        return (np.array(arr) / (2*math.pi) * 4096 + 2048).astype(int)

    def __init__(self, name, state_cb=None, pong_cb=None):
        self.state_cb = state_cb
        self.pong_cb = pong_cb

        self.ser = serial.Serial(name, 921600, timeout=None)
        # self.ser.rts = False

        self.lock = threading.Lock()
        self.last_position = np.array([2048, 2560, 1536, 2048, 2048, 2048, 1200])
        self.last_velocity = np.array([0, 0, 0, 0, 0, 0, 0])
        self.last_effort = np.array([0, 0, 0, 0, 0, 0, 0])
        self.last_time = time.time()

        self.write_lock = threading.Lock()

        self.config_cb = None
        self.config_cb_lock = threading.Lock()

        self.quit = threading.Event()
        self.t = threading.Thread(target=self.recv_thread)
        self.t.daemon = True
        self.t.start()

    def recv_thread(self):
        try:
            while not self.quit.is_set():
                data = self.ser.read(1)
                if not (data[0] == self.COMM_HEAD): # 逐步同步
                    sys.stdout.buffer.write(data)
                    sys.stdout.flush()
                    continue
                
                data += self.ser.read(16 - 1)
                assert(len(data) == 16)

                if data[1] == self.COMM_TYPE_PONG:
                    if self.state_cb is not None:
                        self.pong_cb(struct.unpack('>xxHHHHHHH', data))
                elif data[1] == self.COMM_TYPE_FEEDBACK:
                    position = self.to_si_unit(np.array(struct.unpack('>xxHHHHHHH', data)))
                    this_time = time.time()
                    with self.lock:
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
            self.ser.write(encoded_data)

    def cmd(self, type, data):
        self.write(struct.pack('>BBHHHHHHH', self.COMM_HEAD, type, *data))

    JOINT_MIN = np.array([-math.pi*0.75, -math.pi/2, -math.pi/2, -math.pi/2, -math.pi/2, -math.pi*0.75, -math.pi/2])
    JOINT_MAX = np.array([math.pi*0.75, math.pi/2, math.pi/2, math.pi/2, math.pi/2, math.pi*0.75, math.pi/2])

    def set_pos(self, pos):
        pos_protected = np.minimum(np.maximum(np.array(pos), self.JOINT_MIN), self.JOINT_MAX)
        if all(np.isclose(pos, pos_protected, atol=0.01)):
            pos[6] += 0.3
            
            self.write(struct.pack('>BBHHHHHHH', self.COMM_HEAD, self.COMM_TYPE_CTRL, *self.to_raw_unit(pos)))
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
    
    node.declare_parameter('device', '/dev/ttyUSB0')

    device = node.get_parameter('device').value
    
    logger.info(device)

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
                float(joint_pos["joint_r1"]), 
                float(joint_pos["joint_r2"]), 
                float(joint_pos["joint_r3"]), 
                float(joint_pos["joint_r4"]), 
                float(joint_pos["joint_r5"]), 
                float(joint_pos["joint_r6"]), 
                float(joint_pos["joint_r7l"]), 
                float(joint_pos["joint_r7r"]) 
            ]

            joint_state_publisher.publish(msg)
            time.sleep(0.1)
    t = threading.Thread(target=publish_joint_states)
    t.daemon = True
    t.start()

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

    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
