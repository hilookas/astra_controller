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

np.set_printoptions(precision=4, suppress=True)

# See https://github.com/ros2/rmw/blob/rolling/rmw/include/rmw/qos_profiles.h
# print(rclpy.impl.implementation_singleton.rclpy_implementation.rmw_qos_profile_t.predefined('qos_profile_sensor_data').to_dict())
# print(rclpy.impl.implementation_singleton.rclpy_implementation.rmw_qos_profile_t.predefined('qos_profile_parameters').to_dict())
# TODO evaluate delay impact
qos_profile_sensor_data_reliable = rclpy.qos.QoSProfile(**rclpy.impl.implementation_singleton.rclpy_implementation.rmw_qos_profile_t.predefined('qos_profile_sensor_data').to_dict())
qos_profile_sensor_data_reliable.reliability = 1

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

def main(args=None):
    rclpy.init(args=args)

    node = rclpy.node.Node("teleop_web_node")

    logger = node.get_logger()
    
    teleopoperator = Teleopoperator()
    
    tf_buffer = Buffer()
    tf_listener = TransformListener(tf_buffer, node)
    
    def get_current_eef_pose(side):
        Tsgoal_msg: geometry_msgs.msg.TransformStamped = tf_buffer.lookup_transform('base_link', 'link_ree_teleop' if side == "right" else 'link_lee_teleop', rclpy.time.Time())
        Tsgoal = pt.transform_from_pq(np.array(pq_from_ros_transform(Tsgoal_msg.transform)))
        return Tsgoal
    teleopoperator.on_get_current_eef_pose = get_current_eef_pose
    
    def on_get_eef_pose(eef_link_name, joint_names, initial_joint_states):
        urdf_name = str(Path(get_package_share_directory("astra_description")) / "urdf" / "astra_description_rel.urdf")

        M, Slist, Blist, Mlist, Glist, robot = loadURDF(
            urdf_name, 
            eef_link_name=eef_link_name, 
            actuated_joint_names=joint_names
        )
        
        return mr.FKinBody(M, Slist, initial_joint_states)
    teleopoperator.on_get_eef_pose = on_get_eef_pose
    
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

    def pub_goal_cb(side, Tsgoal):
        pub_T(goal_pose_publisher[side], Tsgoal)
    teleopoperator.on_pub_goal = pub_goal_cb

    def pub_goal_inactive_cb(side, Tsgoal):
        pub_T(goal_pose_inactive_publisher[side], Tsgoal)
    teleopoperator.on_pub_goal_inactive = pub_goal_inactive_cb

    def pub_cam_cb(side, Tsgoal):
        pub_T(cam_pose_publisher[side], Tsgoal)
    teleopoperator.on_pub_cam = pub_cam_cb
    
    def pub_gripper_cb(side, gripper_open):
        arm_gripper_joint_command_publisher[side].publish(astra_controller_interfaces.msg.JointCommand(
            name=["joint_r7r" if side == "right" else "joint_l7r"],
            position_cmd=[gripper_open]
        ))
    teleopoperator.on_pub_gripper = pub_gripper_cb

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
            teleopoperator.webserver.track_feed(name, image)
        return cb
    node.create_subscription(sensor_msgs.msg.Image, 'cam_head/image_raw', get_cb("head"), qos_profile_sensor_data_reliable)
    node.create_subscription(sensor_msgs.msg.Image, 'left/cam_wrist/image_raw', get_cb("wrist_left"), qos_profile_sensor_data_reliable)
    node.create_subscription(sensor_msgs.msg.Image, 'right/cam_wrist/image_raw', get_cb("wrist_right"), qos_profile_sensor_data_reliable)
    # rclpy.qos.qos_profile_sensor_data: best effort reliability and a smaller queue size
    # see: https://docs.ros.org/en/rolling/Concepts/Intermediate/About-Quality-of-Service-Settings.html
    # see: https://blog.csdn.net/qq_38649880/article/details/105908598

    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()