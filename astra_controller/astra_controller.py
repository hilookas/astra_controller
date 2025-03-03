import rclpy
import rclpy.node
import rclpy.qos
import rclpy.action

import sensor_msgs.msg
import std_msgs.msg
import astra_controller_interfaces.msg
import astra_controller_interfaces.srv
import threading
import numpy as np
import time
import geometry_msgs.msg
import nav_msgs.msg

from tf2_ros.transform_listener import TransformListener
from tf2_ros.buffer import Buffer
import tf2_py as tf2

# See https://github.com/ros2/rmw/blob/rolling/rmw/include/rmw/qos_profiles.h
# print(rclpy.impl.implementation_singleton.rclpy_implementation.rmw_qos_profile_t.predefined('qos_profile_sensor_data').to_dict())
# print(rclpy.impl.implementation_singleton.rclpy_implementation.rmw_qos_profile_t.predefined('qos_profile_parameters').to_dict())
# TODO evaluate delay impact
qos_profile_sensor_data_reliable = rclpy.qos.QoSProfile(**rclpy.impl.implementation_singleton.rclpy_implementation.rmw_qos_profile_t.predefined('qos_profile_sensor_data').to_dict())
qos_profile_sensor_data_reliable.reliability = 1

def without_keys(d, keys):
    return {k: v for k, v in d.items() if k not in keys}
        
def pq_from_ros_pose(msg: geometry_msgs.msg.Pose):
    return [
        msg.position.x,
        msg.position.y,
        msg.position.z,
        msg.orientation.w,
        msg.orientation.x,
        msg.orientation.y,
        msg.orientation.z
    ]
        
def pq_from_ros_transform(msg: geometry_msgs.msg.Transform):
    return [
        msg.translation.x,
        msg.translation.y,
        msg.translation.z,
        msg.rotation.w,
        msg.rotation.x,
        msg.rotation.y,
        msg.rotation.z
    ]

class AstraController:
    def __init__(self, space=None):
        rclpy.init()
        
        self.space = space

        self.node = node = rclpy.node.Node('arm_node')

        self.logger = logger = self.node.get_logger()
        
        self.is_quit = threading.Event()
        
        self.reset_buf()
        
        # Register subscriptions
        def cb(msg):
            assert msg.data # True
            self.reset = msg.data # True
        node.create_subscription(
            std_msgs.msg.Bool, 'reset', cb, rclpy.qos.qos_profile_sensor_data 
        )
        
        def cb(msg):
            assert msg.data # True
            self.done = msg.data # True
        node.create_subscription(
            std_msgs.msg.Bool, 'done', cb, rclpy.qos.qos_profile_sensor_data 
        )
        
        def get_cb(name):
            def cb(msg: sensor_msgs.msg.Image):
                assert msg.encoding == "rgb8"
                assert msg.height == 360 and msg.width == 640
                image = np.asarray(msg.data).reshape(msg.height, msg.width, 3) # shape: [360, 640, 3]
                self.images[name] = image
            return cb
        node.create_subscription(sensor_msgs.msg.Image, "cam_head/image_raw", get_cb("head"), qos_profile_sensor_data_reliable)
        node.create_subscription(sensor_msgs.msg.Image, "left/cam_wrist/image_raw", get_cb("wrist_left"), qos_profile_sensor_data_reliable)
        node.create_subscription(sensor_msgs.msg.Image, "right/cam_wrist/image_raw", get_cb("wrist_right"), qos_profile_sensor_data_reliable)

        tf_buffer = Buffer()
        TransformListener(tf_buffer, node)
        
        def cb(msg: sensor_msgs.msg.JointState):
            self.joint_states.update(without_keys(dict(zip(msg.name, msg.position)), ["joint_r7l", "joint_l7l"]))
            
            try:
                if "joint_l6" in msg.name:
                    T_msg: geometry_msgs.msg.TransformStamped = tf_buffer.lookup_transform('base_link', 'link_lee_teleop', rclpy.time.Time())
                    self.joint_states["eef_l"] = pq_from_ros_transform(T_msg.transform)
                if "joint_r6" in msg.name:
                    T_msg: geometry_msgs.msg.TransformStamped = tf_buffer.lookup_transform('base_link', 'link_ree_teleop', rclpy.time.Time())
                    self.joint_states["eef_r"] = pq_from_ros_transform(T_msg.transform)
            except tf2.LookupException:
                pass
            except tf2.ConnectivityException:
                pass
            except tf2.ExtrapolationException:
                pass
        node.create_subscription(sensor_msgs.msg.JointState, "joint_states", cb, rclpy.qos.qos_profile_sensor_data)
        
        def cb(msg: nav_msgs.msg.Odometry):
            self.joint_states["twist_linear"] = msg.twist.twist.linear.x
            self.joint_states["twist_angular"] = msg.twist.twist.angular.z
            
            self.joint_states["odom"] = pq_from_ros_pose(msg.pose.pose)
        node.create_subscription(nav_msgs.msg.Odometry, "odom", cb, rclpy.qos.qos_profile_sensor_data)
        
        def cb(msg: astra_controller_interfaces.msg.JointCommand):
            self.joint_commands.update(without_keys(dict(zip(msg.name, msg.position_cmd)), ["joint_r7l", "joint_l7l"]))
        node.create_subscription(astra_controller_interfaces.msg.JointCommand, "left/lift/joint_command", cb, rclpy.qos.qos_profile_sensor_data)
        node.create_subscription(astra_controller_interfaces.msg.JointCommand, "left/arm/joint_command", cb, rclpy.qos.qos_profile_sensor_data)
        node.create_subscription(astra_controller_interfaces.msg.JointCommand, "left/arm/gripper_joint_command", cb, rclpy.qos.qos_profile_sensor_data)
        node.create_subscription(astra_controller_interfaces.msg.JointCommand, "right/lift/joint_command", cb, rclpy.qos.qos_profile_sensor_data)
        node.create_subscription(astra_controller_interfaces.msg.JointCommand, "right/arm/joint_command", cb, rclpy.qos.qos_profile_sensor_data)
        node.create_subscription(astra_controller_interfaces.msg.JointCommand, "right/arm/gripper_joint_command", cb, rclpy.qos.qos_profile_sensor_data)
        node.create_subscription(astra_controller_interfaces.msg.JointCommand, "head/joint_command", cb, rclpy.qos.qos_profile_sensor_data)
        
        def get_cb(name):
            def cb(msg: geometry_msgs.msg.PoseStamped):
                self.joint_commands[name] = pq_from_ros_pose(msg.pose)
            return cb
        node.create_subscription(geometry_msgs.msg.PoseStamped, "left/goal_pose", get_cb("eef_l"), rclpy.qos.qos_profile_sensor_data)
        node.create_subscription(geometry_msgs.msg.PoseStamped, "right/goal_pose", get_cb("eef_r"), rclpy.qos.qos_profile_sensor_data)

        def cb(msg: geometry_msgs.msg.Twist):
            self.joint_commands["twist_linear"] = msg.linear.x
            self.joint_commands["twist_angular"] = msg.angular.z
        node.create_subscription(geometry_msgs.msg.Twist, "cmd_vel", cb, rclpy.qos.qos_profile_sensor_data)
        
        self.left_arm_joint_command_publisher = node.create_publisher(astra_controller_interfaces.msg.JointCommand, f"left/arm/joint_command", 10)
        self.left_lift_joint_command_publisher = node.create_publisher(astra_controller_interfaces.msg.JointCommand, f"left/lift/joint_command", 10)
        self.left_arm_gripper_joint_command_publisher = node.create_publisher(astra_controller_interfaces.msg.JointCommand, f"left/arm/gripper_joint_command", 10)
        
        self.right_arm_joint_command_publisher = node.create_publisher(astra_controller_interfaces.msg.JointCommand, f"right/arm/joint_command", 10)
        self.right_lift_joint_command_publisher = node.create_publisher(astra_controller_interfaces.msg.JointCommand, f"right/lift/joint_command", 10)
        self.right_arm_gripper_joint_command_publisher = node.create_publisher(astra_controller_interfaces.msg.JointCommand, f"right/arm/gripper_joint_command", 10)
        
        self.head_joint_command_publisher = node.create_publisher(astra_controller_interfaces.msg.JointCommand, f"head/joint_command", 10)
        
        self.left_goal_pose_publisher = node.create_publisher(geometry_msgs.msg.PoseStamped, f"left/goal_pose", 10)
        self.right_goal_pose_publisher = node.create_publisher(geometry_msgs.msg.PoseStamped, f"right/goal_pose", 10)
        
        self.cmd_vel_publisher = node.create_publisher(geometry_msgs.msg.Twist, 'cmd_vel', 10)
        
        self.t = threading.Thread(target=rclpy.spin, args=(self.node,), daemon=True).start()
    
    def reset_buf(self):
        self.reset = False
        self.done = False

        self.images = {
            "head": None, # np.zeros((360, 640, 3), np.uint8),
            "wrist_left": None, # np.zeros((360, 640, 3), np.uint8),
            "wrist_right": None, # np.zeros((360, 640, 3), np.uint8)
        }
        
        self.joint_states = {
            "joint_l1": None, "joint_l2": None, "joint_l3": None, "joint_l4": None, "joint_l5": None, "joint_l6": None, "joint_l7r": None,
            "joint_r1": None, "joint_r2": None, "joint_r3": None, "joint_r4": None, "joint_r5": None, "joint_r6": None, "joint_r7r": None,
            "joint_head_pan": None, "joint_head_tilt": None,
            "twist_linear": None, "twist_angular": None, 
            "eef_l": None, # [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            "eef_r": None, # [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            "odom": None, # [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        }
        
        self.joint_commands = {
            "joint_l1": None, "joint_l2": None, "joint_l3": None, "joint_l4": None, "joint_l5": None, "joint_l6": None, "joint_l7r": None,
            "joint_r1": None, "joint_r2": None, "joint_r3": None, "joint_r4": None, "joint_r5": None, "joint_r6": None, "joint_r7r": None,
            "joint_head_pan": None, "joint_head_tilt": None,
            "twist_linear": None, "twist_angular": None, 
            "eef_l": None, # [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            "eef_r": None, # [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        }
    
    def states_ready(self):
        for k, v in self.images.items():
            if v is None:
                print(f"Waiting for image {k} to be ready")
                return False
        
        for k, v in self.joint_states.items():
            if v is None:
                print(f"Waiting for state {k} to be ready")
                return False
        
        return True
    
    def commands_ready(self):
        for k, v in self.joint_commands.items():
            if v is None:
                print(f"Waiting for command {k} to be ready")
                return False
        
        return True

    def wait_for_reset(self):
        print("Waiting for reset")
        while not self.reset:
            time.sleep(0.1)
        
        self.reset_buf()
        
        while not self.states_ready():
            time.sleep(0.1)
        
    def connect(self):            
        print("connected")
        
    def disconnect(self):
        print("disconnect")
        
    def read_leader_present_position(self):
        while not self.commands_ready():
            time.sleep(0.1)

        if self.space == "joint":
            action = [self.joint_commands[key] for key in [
                "joint_l1", "joint_l2", "joint_l3", "joint_l4", "joint_l5", "joint_l6",
                "joint_l7r",
                "joint_r1", "joint_r2", "joint_r3", "joint_r4", "joint_r5", "joint_r6",
                "joint_r7r",
                "twist_linear", "twist_angular", 
                "joint_head_pan", "joint_head_tilt",
            ]]
        elif self.space == "cartesian":
            action = self.joint_commands["eef_l"] + self.joint_commands["eef_r"] + [self.joint_commands[key] for key in [
                "joint_l7r",
                "joint_r7r",
                "twist_linear", "twist_angular", 
                "joint_head_pan", "joint_head_tilt",
            ]]
        else:
            action = None

        return action, [self.joint_commands[key] for key in [
            "joint_l1", "joint_l2", "joint_l3", "joint_l4", "joint_l5", "joint_l6",
        ]], [self.joint_commands[key] for key in [
            "joint_l7r",
        ]], [self.joint_commands[key] for key in [
            "joint_r1", "joint_r2", "joint_r3", "joint_r4", "joint_r5", "joint_r6",
        ]], [self.joint_commands[key] for key in [
            "joint_r7r",
        ]], [self.joint_commands[key] for key in [
            "twist_linear", "twist_angular", 
        ]], self.joint_commands["eef_l"], self.joint_commands["eef_r"], [self.joint_commands[key] for key in [
            "joint_head_pan", "joint_head_tilt",
        ]]
    
    def read_present_position(self):
        if self.space == "joint":
            observation = [self.joint_states[key] for key in [
                "joint_l1", "joint_l2", "joint_l3", "joint_l4", "joint_l5", "joint_l6",
                "joint_l7r",
                "joint_r1", "joint_r2", "joint_r3", "joint_r4", "joint_r5", "joint_r6",
                "joint_r7r",
                "twist_linear", "twist_angular", 
                "joint_head_pan", "joint_head_tilt",
            ]]
        elif self.space == "cartesian":
            observation = self.joint_states["eef_l"] + self.joint_states["eef_r"] + [self.joint_states[key] for key in [
                "joint_l7r",
                "joint_r7r",
                "twist_linear", "twist_angular", 
                "joint_head_pan", "joint_head_tilt",
            ]]
        else:
            observation = None
        
        return observation, [self.joint_states[key] for key in [
            "joint_l1", "joint_l2", "joint_l3", "joint_l4", "joint_l5", "joint_l6",
        ]], [self.joint_states[key] for key in [
            "joint_l7r",
        ]], [self.joint_states[key] for key in [
            "joint_r1", "joint_r2", "joint_r3", "joint_r4", "joint_r5", "joint_r6",
        ]], [self.joint_states[key] for key in [
            "joint_r7r",
        ]], [self.joint_states[key] for key in [
            "twist_linear", "twist_angular", 
        ]], self.joint_states["eef_l"], self.joint_states["eef_r"], self.joint_states["odom"], [self.joint_states[key] for key in [
            "joint_head_pan", "joint_head_tilt",
        ]]
        
    def write_goal_position(self, goal_pos: list[float]): # TODO support IK mode
        if self.space == "joint":
            joint_commands = {}
            joint_commands.update(dict(zip([
                "joint_l1", "joint_l2", "joint_l3", "joint_l4", "joint_l5", "joint_l6",
                "joint_l7r",
                "joint_r1", "joint_r2", "joint_r3", "joint_r4", "joint_r5", "joint_r6",
                "joint_r7r",
                "twist_linear", "twist_angular", 
                "joint_head_pan", "joint_head_tilt",
            ], goal_pos)))
        
            self.left_arm_joint_command_publisher.publish(astra_controller_interfaces.msg.JointCommand(
                name=[ "joint_l2", "joint_l3", "joint_l4", "joint_l5", "joint_l6", ],
                position_cmd=[joint_commands[key] for key in [ "joint_l2", "joint_l3", "joint_l4", "joint_l5", "joint_l6", ]]
            ))

            self.left_lift_joint_command_publisher.publish(astra_controller_interfaces.msg.JointCommand(
                name=[ "joint_l1", ],
                position_cmd=[joint_commands[key] for key in [ "joint_l1", ]]
            ))

            self.left_arm_gripper_joint_command_publisher.publish(astra_controller_interfaces.msg.JointCommand(
                name=[ "joint_l7r", ],
                position_cmd=[joint_commands[key] for key in [ "joint_l7r", ]]
            ))
            
            self.right_arm_joint_command_publisher.publish(astra_controller_interfaces.msg.JointCommand(
                name=[ "joint_r2", "joint_r3", "joint_r4", "joint_r5", "joint_r6", ],
                position_cmd=[joint_commands[key] for key in [ "joint_r2", "joint_r3", "joint_r4", "joint_r5", "joint_r6", ]]
            ))

            self.right_lift_joint_command_publisher.publish(astra_controller_interfaces.msg.JointCommand(
                name=[ "joint_r1", ],
                position_cmd=[joint_commands[key] for key in [ "joint_r1", ]]
            ))

            self.right_arm_gripper_joint_command_publisher.publish(astra_controller_interfaces.msg.JointCommand(
                name=[ "joint_r7r", ],
                position_cmd=[joint_commands[key] for key in [ "joint_r7r", ]]
            ))
            
            self.head_joint_command_publisher.publish(astra_controller_interfaces.msg.JointCommand(
                name=[ "joint_head_pan", "joint_head_tilt", ],
                position_cmd=[joint_commands[key] for key in [ "joint_head_pan", "joint_head_tilt", ]]
            ))
            
            msg = geometry_msgs.msg.Twist()
            msg.linear.x = joint_commands["twist_linear"]
            msg.angular.z = joint_commands["twist_angular"]
            # self.cmd_vel_publisher.publish(msg)
        elif self.space == "cartesian":
            raise NotImplementedError("Cartesian space is not supported for now")
        else:
            raise Exception("Give a space to the AstraController!")
        
    def read_cameras(self):
        return self.images
        
    def quit(self):
        if not self.is_quit.is_set():
            self.is_quit.set()

            self.node.destroy_node()
            rclpy.shutdown()

    def __del__(self):
        pass
