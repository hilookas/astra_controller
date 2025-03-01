import rclpy
import rclpy.node
import rclpy.publisher

from astra_teleop_web.teleoprator import Teleopoperator

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

import modern_robotics as mr
from mr_urdf_loader import loadURDF
from pathlib import Path
from ament_index_python import get_package_share_directory

from astra_controller.astra_controller import pq_from_ros_transform

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

    node = rclpy.node.Node("teleop_web_node")

    logger = node.get_logger()
    
    teleopoperator = Teleopoperator()
    
    tf_buffer = Buffer()
    tf_listener = TransformListener(tf_buffer, node)
    
    def get_current_eef_pose_cb(side):
        Tsgoal_msg: geometry_msgs.msg.TransformStamped = tf_buffer.lookup_transform('base_link', 'link_ree_teleop' if side == "right" else 'link_lee_teleop', rclpy.time.Time())
        Tsgoal = pt.transform_from_pq(np.array(pq_from_ros_transform(Tsgoal_msg.transform)))
        return Tsgoal
    teleopoperator.on_get_current_eef_pose = get_current_eef_pose_cb
    
    M = {}
    Slist = {}
    
    for side in ["left", "right"]:
        urdf_name = str(Path(get_package_share_directory("astra_description")) / "urdf" / "astra_description_rel.urdf")

        M[side], Slist[side], _, _, _, _ = loadURDF(
            urdf_name, 
            eef_link_name='link_ree_teleop' if side == "right" else 'link_lee_teleop', 
            actuated_joint_names=["joint_r1", "joint_r2", "joint_r3", "joint_r4", "joint_r5", "joint_r6" ] if side == "right" else ["joint_l1", "joint_l2", "joint_l3", "joint_l4", "joint_l5", "joint_l6" ]
        )

    def get_initial_eef_pose_cb(side, initial_joint_states):
        return mr.FKinSpace(M[side], Slist[side], initial_joint_states)
    teleopoperator.on_get_initial_eef_pose = get_initial_eef_pose_cb
    
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
    teleopoperator.on_pub_goal = pub_goal_cb
    
    def pub_gripper_cb(side, gripper_open):
        arm_gripper_joint_command_publisher[side].publish(astra_controller_interfaces.msg.JointCommand(
            name=["joint_r7r" if side == "right" else "joint_l7r"],
            position_cmd=[gripper_open]
        ))
    teleopoperator.on_pub_gripper = pub_gripper_cb

    head_joint_command_publisher = node.create_publisher(astra_controller_interfaces.msg.JointCommand, "head/joint_command", 10)

    def pub_head_cb(head_pan, head_tilt):
        head_joint_command_publisher.publish(astra_controller_interfaces.msg.JointCommand(
            name=["joint_head_pan", "joint_head_tilt"],
            position_cmd=[head_pan, head_tilt]
        ))
    teleopoperator.on_pub_head = pub_head_cb

    cmd_vel_publisher = node.create_publisher(geometry_msgs.msg.Twist, 'cmd_vel', 10)
    def cmd_vel_cb(linear_vel, angular_vel):
        msg = geometry_msgs.msg.Twist()
        msg.linear.x = linear_vel
        msg.angular.z = angular_vel
        cmd_vel_publisher.publish(msg)
    teleopoperator.on_cmd_vel = cmd_vel_cb
    
    reset_publisher = node.create_publisher(std_msgs.msg.Bool, 'reset', 10)
    done_publisher = node.create_publisher(std_msgs.msg.Bool, 'done', 10)

    def reset_cb():
        reset_publisher.publish(std_msgs.msg.Bool(data=True))
    teleopoperator.on_reset = reset_cb

    def done_cb():
        done_publisher.publish(std_msgs.msg.Bool(data=True))
    teleopoperator.on_done = done_cb

    def get_cb(name):
        def cb(msg: sensor_msgs.msg.Image):
            assert msg.encoding == "rgb8"
            assert msg.height == 360 and msg.width == 640
            image = np.asarray(msg.data).reshape(msg.height, msg.width, 3)
            teleopoperator.webserver.track_feed(name, (image, msg.header.stamp.sec, msg.header.stamp.nanosec))
        return cb
    node.create_subscription(sensor_msgs.msg.Image, 'cam_head/image_raw', get_cb("head"), qos_profile_sensor_data_reliable)
    node.create_subscription(sensor_msgs.msg.Image, 'left/cam_wrist/image_raw', get_cb("wrist_left"), qos_profile_sensor_data_reliable)
    node.create_subscription(sensor_msgs.msg.Image, 'right/cam_wrist/image_raw', get_cb("wrist_right"), qos_profile_sensor_data_reliable)

    # Register IK Failed subscriptions
    def cb(msg):
        teleopoperator.error_cb(f"[IK Left]\n{msg.data}")
    node.create_subscription(
        std_msgs.msg.String, 'left/ik_error', cb, rclpy.qos.qos_profile_sensor_data 
    )
    
    def cb(msg):
        teleopoperator.error_cb(f"[IK Right]\n{msg.data}")
    node.create_subscription(
        std_msgs.msg.String, 'right/ik_error', cb, rclpy.qos.qos_profile_sensor_data 
    )
    
    def cb(msg):
        teleopoperator.error_cb(f"[Arm Left]\n{msg.data}")
    node.create_subscription(
        std_msgs.msg.String, 'left/arm/error', cb, rclpy.qos.qos_profile_sensor_data 
    )
    
    def cb(msg):
        teleopoperator.error_cb(f"[Arm Right]\n{msg.data}")
    node.create_subscription(
        std_msgs.msg.String, 'right/arm/error', cb, rclpy.qos.qos_profile_sensor_data 
    )
    
    def cb(msg):
        teleopoperator.error_cb(f"[Lift Left]\n{msg.data}")
    node.create_subscription(
        std_msgs.msg.String, 'left/lift/error', cb, rclpy.qos.qos_profile_sensor_data 
    )
    
    def cb(msg):
        teleopoperator.error_cb(f"[Lift Right]\n{msg.data}")
    node.create_subscription(
        std_msgs.msg.String, 'right/lift/error', cb, rclpy.qos.qos_profile_sensor_data 
    )

    try:
        rclpy.spin(node)
    except KeyboardInterrupt as err:
        teleopoperator.webserver.loop.stop()
        raise err

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()