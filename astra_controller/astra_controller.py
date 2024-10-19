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
    def __init__(self, space="both"):
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
                image = np.asarray(msg.data).reshape(msg.height, msg.width, 3) # shape: [720, 1280, 3]
                if name == "head":
                    # assert msg.height == 720 and msg.width == 1280
                    assert msg.height == 360 and msg.width == 640
                    # image = cv2.resize(image, (1280, 720))
                else:
                    assert msg.height == 360 and msg.width == 640
                    # image = cv2.resize(image, (640, 360))
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
                    self.joint_commands["eef_l"] = pq_from_ros_transform(T_msg.transform)
                if "joint_r6" in msg.name:
                    T_msg: geometry_msgs.msg.TransformStamped = tf_buffer.lookup_transform('base_link', 'link_ree_teleop', rclpy.time.Time())
                    self.joint_commands["eef_r"] = pq_from_ros_transform(T_msg.transform)
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
        
        def get_cb(names):
            def cb(msg: astra_controller_interfaces.msg.JointGroupCommand):
                self.joint_commands.update(without_keys(dict(zip(names, msg.cmd)), ["joint_r7l", "joint_l7l"]))
            return cb
        node.create_subscription(astra_controller_interfaces.msg.JointGroupCommand, "left/lift/joint_command", get_cb(["joint_l1"]), rclpy.qos.qos_profile_sensor_data)
        node.create_subscription(astra_controller_interfaces.msg.JointGroupCommand, "left/arm/joint_command", get_cb(["joint_l2", "joint_l3", "joint_l4", "joint_l5", "joint_l6"]), rclpy.qos.qos_profile_sensor_data)
        node.create_subscription(astra_controller_interfaces.msg.JointGroupCommand, "left/arm/gripper_joint_command", get_cb(["joint_l7r"]), rclpy.qos.qos_profile_sensor_data)
        node.create_subscription(astra_controller_interfaces.msg.JointGroupCommand, "right/lift/joint_command", get_cb(["joint_r1"]), rclpy.qos.qos_profile_sensor_data)
        node.create_subscription(astra_controller_interfaces.msg.JointGroupCommand, "right/arm/joint_command", get_cb(["joint_r2", "joint_r3", "joint_r4", "joint_r5", "joint_r6"]), rclpy.qos.qos_profile_sensor_data)
        node.create_subscription(astra_controller_interfaces.msg.JointGroupCommand, "right/arm/gripper_joint_command", get_cb(["joint_r7r"]), rclpy.qos.qos_profile_sensor_data)
        
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
        
        self.left_arm_joint_command_publisher = node.create_publisher(astra_controller_interfaces.msg.JointGroupCommand, f"left/arm/joint_command", 10)
        self.left_lift_joint_command_publisher = node.create_publisher(astra_controller_interfaces.msg.JointGroupCommand, f"left/lift/joint_command", 10)
        self.left_arm_gripper_joint_command_publisher = node.create_publisher(astra_controller_interfaces.msg.JointGroupCommand, f"left/arm/gripper_joint_command", 10)
        
        self.right_arm_joint_command_publisher = node.create_publisher(astra_controller_interfaces.msg.JointGroupCommand, f"right/arm/joint_command", 10)
        self.right_lift_joint_command_publisher = node.create_publisher(astra_controller_interfaces.msg.JointGroupCommand, f"right/lift/joint_command", 10)
        self.right_arm_gripper_joint_command_publisher = node.create_publisher(astra_controller_interfaces.msg.JointGroupCommand, f"right/arm/gripper_joint_command", 10)
        
        self.cmd_vel_publisher = node.create_publisher(geometry_msgs.msg.Twist, 'cmd_vel', 10)
        
        self.t = threading.Thread(target=rclpy.spin, args=(self.node,), daemon=True).start()
    
    def reset_buf(self):
        self.reset = False
        self.done = False

        self.images = {
            # "head": np.zeros((720, 1280, 3), np.uint8),
            "head": np.zeros((360, 640, 3), np.uint8),
            "wrist_left": np.zeros((360, 640, 3), np.uint8),
            "wrist_right": np.zeros((360, 640, 3), np.uint8)
        }
        
        self.joint_states = {
            "joint_l1": 0.0, "joint_l2": 0.0, "joint_l3": 0.0, "joint_l4": 0.0, "joint_l5": 0.0, "joint_l6": 0.0, "joint_l7r": 0.0,
            "joint_r1": 0.0, "joint_r2": 0.0, "joint_r3": 0.0, "joint_r4": 0.0, "joint_r5": 0.0, "joint_r6": 0.0, "joint_r7r": 0.0,
            "twist_linear": 0.0, "twist_angular": 0.0, 
            "eef_l": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], "eef_r": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            "odom": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        }
        
        self.joint_commands = {
            "joint_l1": 0.0, "joint_l2": 0.0, "joint_l3": 0.0, "joint_l4": 0.0, "joint_l5": 0.0, "joint_l6": 0.0, "joint_l7r": 0.0,
            "joint_r1": 0.0, "joint_r2": 0.0, "joint_r3": 0.0, "joint_r4": 0.0, "joint_r5": 0.0, "joint_r6": 0.0, "joint_r7r": 0.0,
            "twist_linear": 0.0, "twist_angular": 0.0, 
            "eef_l": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], "eef_r": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        }
    
    def wait_for_reset(self):
        print("Waiting for reset")
        while not self.reset:
            time.sleep(0.5)
        self.reset_buf()
        
    def connect(self):            
        print("connected")
        
    def disconnect(self):
        print("disconnect")
        
    def read_leader_present_position(self):
        if self.space == 'joint':
            return [self.joint_commands[key] for key in [
                "joint_l1", "joint_l2", "joint_l3", "joint_l4", "joint_l5", "joint_l6",
                "joint_l7r",
                "joint_r1", "joint_r2", "joint_r3", "joint_r4", "joint_r5", "joint_r6",
                "joint_r7r",
                "twist_linear", "twist_angular", 
            ]]
        elif self.space == 'cartesian':
            return self.joint_commands["eef_l"] + self.joint_commands["eef_r"] + [self.joint_commands[key] for key in [
                "joint_l7r",
                "joint_r7r",
                "twist_linear", "twist_angular", 
            ]]
        elif self.space == 'both': # both
            return [self.joint_commands[key] for key in [
                "joint_l1", "joint_l2", "joint_l3", "joint_l4", "joint_l5", "joint_l6",
            ]], [self.joint_commands[key] for key in [
                "joint_l7r",
            ]], [self.joint_commands[key] for key in [
                "joint_r1", "joint_r2", "joint_r3", "joint_r4", "joint_r5", "joint_r6",
            ]], [self.joint_commands[key] for key in [
                "joint_r7r",
            ]], [self.joint_commands[key] for key in [
                "twist_linear", "twist_angular", 
            ]], self.joint_commands["eef_l"], self.joint_commands["eef_r"]
        else:
            raise Exception("Specify data space!")
    
    def read_present_position(self):
        if self.space == 'joint':
            return [self.joint_states[key] for key in [
                "joint_l1", "joint_l2", "joint_l3", "joint_l4", "joint_l5", "joint_l6",
                "joint_l7r",
                "joint_r1", "joint_r2", "joint_r3", "joint_r4", "joint_r5", "joint_r6",
                "joint_r7r",
                "twist_linear", "twist_angular", 
            ]]
        elif self.space == 'cartesian':
            return self.joint_states["eef_l"] + self.joint_states["eef_r"] + [self.joint_states[key] for key in [
                "joint_l7r",
                "joint_r7r",
                "twist_linear", "twist_angular", 
            ]]
        elif self.space == 'both': # both
            return [self.joint_states[key] for key in [
                "joint_l1", "joint_l2", "joint_l3", "joint_l4", "joint_l5", "joint_l6",
            ]], [self.joint_states[key] for key in [
                "joint_l7r",
            ]], [self.joint_states[key] for key in [
                "joint_r1", "joint_r2", "joint_r3", "joint_r4", "joint_r5", "joint_r6",
            ]], [self.joint_states[key] for key in [
                "joint_r7r",
            ]], [self.joint_states[key] for key in [
                "twist_linear", "twist_angular", 
            ]], self.joint_states["eef_l"], self.joint_states["eef_r"], self.joint_states["odom"]
        else:
            raise Exception("Specify data space!")
        
    def write_goal_position(self, goal_pos: list[float]): # TODO support IK mode
        if self.space == 'joint':
            joint_commands = {}
            joint_commands.update(dict(zip([
                "joint_l1", "joint_l2", "joint_l3", "joint_l4", "joint_l5", "joint_l6",
                "joint_l7r",
                "joint_r1", "joint_r2", "joint_r3", "joint_r4", "joint_r5", "joint_r6",
                "joint_r7r",
                "twist_linear", "twist_angular", 
            ], goal_pos)))
        
            self.left_arm_joint_command_publisher.publish(astra_controller_interfaces.msg.JointGroupCommand(
                cmd=[joint_commands[key] for key in [
                    "joint_l2", "joint_l3", "joint_l4", "joint_l5", "joint_l6",
                ]]
            ))

            self.left_lift_joint_command_publisher.publish(astra_controller_interfaces.msg.JointGroupCommand(
                cmd=[joint_commands[key] for key in [
                    "joint_l1"
                ]]
            ))

            self.left_arm_gripper_joint_command_publisher.publish(astra_controller_interfaces.msg.JointGroupCommand(
                cmd=[joint_commands[key] for key in [
                    "joint_l7r",
                ]]
            ))
            

            self.right_arm_joint_command_publisher.publish(astra_controller_interfaces.msg.JointGroupCommand(
                cmd=[joint_commands[key] for key in [
                    "joint_r2", "joint_r3", "joint_r4", "joint_r5", "joint_r6",
                ]]
            ))

            self.right_lift_joint_command_publisher.publish(astra_controller_interfaces.msg.JointGroupCommand(
                cmd=[joint_commands[key] for key in [
                    "joint_r1"
                ]]
            ))

            self.right_arm_gripper_joint_command_publisher.publish(astra_controller_interfaces.msg.JointGroupCommand(
                cmd=[joint_commands[key] for key in [
                    "joint_r7r",
                ]]
            ))
            
            
            msg = geometry_msgs.msg.Twist()
            msg.linear.x = joint_commands["twist_linear"]
            msg.angular.z = joint_commands["twist_angular"]
            self.cmd_vel_publisher.publish(msg)
        else:
            raise Exception("Not implemented!")
        
    def read_cameras(self):
        return self.images
        
    def quit(self):
        if not self.is_quit.is_set():
            self.is_quit.set()

            self.node.destroy_node()
            rclpy.shutdown()

    def __del__(self):
        pass
