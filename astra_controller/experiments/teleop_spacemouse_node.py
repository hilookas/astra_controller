import rclpy
import rclpy.node
import rclpy.publisher

import sensor_msgs.msg

import rclpy
import rclpy.node
import rclpy.qos

import std_msgs.msg
import geometry_msgs.msg
import astra_controller_interfaces.msg

from tf2_ros.transform_listener import TransformListener
from tf2_ros.buffer import Buffer

import numpy as np
from pytransform3d import transformations as pt
from pytransform3d import rotations as pr

import modern_robotics as mr
from mr_urdf_loader import loadURDF
from pathlib import Path
from ament_index_python import get_package_share_directory

from astra_controller.experiments.spacemouse_agent import SpacemouseAgent
from astra_controller.astra_controller import pq_from_ros_transform
from astra_teleop_web.teleoprator import GRIPPER_MAX

import math
import time
np.set_printoptions(precision=4, suppress=True)

# See https://github.com/ros2/rmw/blob/rolling/rmw/include/rmw/qos_profiles.h
# print(rclpy.impl.implementation_singleton.rclpy_implementation.rmw_qos_profile_t.predefined('qos_profile_sensor_data').to_dict())
# print(rclpy.impl.implementation_singleton.rclpy_implementation.rmw_qos_profile_t.predefined('qos_profile_parameters').to_dict())
# TODO evaluate delay impact
# rclpy.qos.qos_profile_sensor_data: best effort reliability and a smaller queue size
# see: https://docs.ros.org/en/rolling/Concepts/Intermediate/About-Quality-of-Service-Settings.html
# see: https://blog.csdn.net/qq_38649880/article/details/105908598
qos_profile_sensor_data_reliable = rclpy.qos.QoSProfile(**rclpy.impl.implementation_singleton.rclpy_implementation.rmw_qos_profile_t.predefined('qos_profile_sensor_data').to_dict())
qos_profile_sensor_data_reliable.reliability = 1

def main(args=None):
    rclpy.init(args=args)

    node = rclpy.node.Node("teleop_spacemouse_node")

    logger = node.get_logger()
    
    def pub_T(pub: rclpy.publisher.Publisher, T, frame_id='base_link'):
        msg = geometry_msgs.msg.PoseStamped()
        msg.header.frame_id = frame_id
        msg.header.stamp = node.get_clock().now().to_msg()
        pq = pt.pq_from_transform(T)
        msg.pose.position.x = pq[0]
        msg.pose.position.y = pq[1]
        msg.pose.position.z = pq[2]
        msg.pose.orientation.w = pq[3]
        msg.pose.orientation.x = pq[4]
        msg.pose.orientation.y = pq[5]
        msg.pose.orientation.z = pq[6]
        pub.publish(msg)
    
    goal_pose_publisher = {}
    goal_pose_inactive_publisher = {}
    cam_pose_publisher = {}
    arm_gripper_joint_command_publisher = {}
    arm_joint_command_publisher = {}
    lift_joint_command_publisher = {}
    
    for side in ["left", "right"]:
        goal_pose_publisher[side] = node.create_publisher(geometry_msgs.msg.PoseStamped, f"{side}/goal_pose", 10)
        goal_pose_inactive_publisher[side] = node.create_publisher(geometry_msgs.msg.PoseStamped, f"{side}/goal_pose_inactive", 10)
        cam_pose_publisher[side] = node.create_publisher(geometry_msgs.msg.PoseStamped, f"{side}/cam_pose", 10)

        arm_gripper_joint_command_publisher[side] = node.create_publisher(astra_controller_interfaces.msg.JointCommand, f"{side}/arm/gripper_joint_command", 10)
        arm_joint_command_publisher[side] = node.create_publisher(astra_controller_interfaces.msg.JointCommand, f"{side}/arm/joint_command", 10)
        lift_joint_command_publisher[side] = node.create_publisher(astra_controller_interfaces.msg.JointCommand, f"{side}/lift/joint_command", 10)
    
    def pub_goal_cb(side, Tsgoal, Tscam=None, Tsgoal_inactive=None):
        if Tsgoal is not None:
            pub_T(goal_pose_publisher[side], Tsgoal)
        if Tsgoal_inactive is not None:
            pub_T(goal_pose_inactive_publisher[side], Tsgoal_inactive)
        if Tscam is not None:
            pub_T(cam_pose_publisher[side], Tscam)
    
    def pub_gripper_cb(side, gripper_open):
        arm_gripper_joint_command_publisher[side].publish(astra_controller_interfaces.msg.JointCommand(
            name=["joint_r7r" if side == "right" else "joint_l7r"],
            position_cmd=[gripper_open]
        ))
    
    tf_buffer = Buffer()
    tf_listener = TransformListener(tf_buffer, node)
    
    # Wait until the transform from 'base_link' to 'link_r6' becomes available.
    while rclpy.ok():
        try:
            Tsgoal_msg: geometry_msgs.msg.TransformStamped = tf_buffer.lookup_transform('base_link', 'link_ree_teleop', rclpy.time.Time())
            break
        except Exception as e:
            logger.warn(str(e))
            # logger.warn("Transform lookup failed (base_link may not exist yet). Retrying...")
            rclpy.spin_once(node)

    Tsgoal = pt.transform_from_pq(np.array(pq_from_ros_transform(Tsgoal_msg.transform)))
    goal_gripper = GRIPPER_MAX
    
    # import IPython; IPython.embed()

    agent = SpacemouseAgent()
    
    control_T = 0.01
    
    while True:
        action, buttons = agent.act(None)
        
        action[:3] *= 200.0 / 1000.0 * control_T # mm/s
        action[3:] *= 60 / 180 * math.pi * control_T # deg/s
        
        Tv = pt.transform_from(pr.matrix_from_euler(action[3:], 0, 1, 2, True), action[:3])
        
        # # Transform frame: cam
        # Tsgoal = Tsgoal @ Tv
        
        # Transform frame: base
        R = Tsgoal[:3, :3]
        p = Tsgoal[:3, 3]
        Tsgoal = pt.transform_from(np.eye(3), p) @ Tv @ pt.transform_from(R, np.zeros(3))
        
        if buttons[0]:
            goal_gripper += 100.0 / 1000.0 * control_T
            if goal_gripper > GRIPPER_MAX:
                goal_gripper = GRIPPER_MAX
        elif buttons[1]:
            goal_gripper -= 100.0 / 1000.0 * control_T
            if goal_gripper < 0:
                goal_gripper = 0

        pub_goal_cb("right", Tsgoal)
        pub_gripper_cb("right", goal_gripper)
        
        time.sleep(control_T)

    rclpy.spin(node)
    
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()