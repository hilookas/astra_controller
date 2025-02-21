import rclpy
import rclpy.node
import rclpy.publisher

import rclpy
import rclpy.node
import rclpy.qos

import geometry_msgs.msg
import astra_controller_interfaces.msg

from tf2_ros.transform_listener import TransformListener
from tf2_ros.buffer import Buffer

import numpy as np
from pytransform3d import transformations as pt
from pytransform3d import rotations as pr

import modern_robotics as mr
from mr_urdf_loader import loadURDF
from matplotlib import pyplot as plt
from urchin import URDF

from astra_controller.experiments.leader_arm_controller import ArmController
from astra_controller.astra_controller import pq_from_ros_transform
from astra_teleop_web.teleoprator import GRIPPER_MAX

import math
import time
np.set_printoptions(precision=4, suppress=True)

from pytransform3d.plot_utils import Frame

# See https://github.com/ros2/rmw/blob/rolling/rmw/include/rmw/qos_profiles.h
# print(rclpy.impl.implementation_singleton.rclpy_implementation.rmw_qos_profile_t.predefined('qos_profile_sensor_data').to_dict())
# print(rclpy.impl.implementation_singleton.rclpy_implementation.rmw_qos_profile_t.predefined('qos_profile_parameters').to_dict())
# TODO evaluate delay impact
# rclpy.qos.qos_profile_sensor_data: best effort reliability and a smaller queue size
# see: https://docs.ros.org/en/rolling/Concepts/Intermediate/About-Quality-of-Service-Settings.html
# see: https://blog.csdn.net/qq_38649880/article/details/105908598
qos_profile_sensor_data_reliable = rclpy.qos.QoSProfile(**rclpy.impl.implementation_singleton.rclpy_implementation.rmw_qos_profile_t.predefined('qos_profile_sensor_data').to_dict())
qos_profile_sensor_data_reliable.reliability = 1

def clip(x, mn, mx):
    return max(mn, min(x, mx))

def main(args=None):
    rclpy.init(args=args)

    node = rclpy.node.Node("teleop_leader_arm_node")

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

    Tsinitgoal = pt.transform_from_pq(np.array(pq_from_ros_transform(Tsgoal_msg.transform)))
    
    # import IPython; IPython.embed()
    
    urdf_name = "urdf/Athena.urdf"
    joint_names = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
    eef_link_name = 'link_eef'

    M, Slist, _, _, _, _ = loadURDF(
        urdf_name, 
        eef_link_name=eef_link_name, 
        actuated_joint_names=joint_names
    )

    # robot = URDF.load(urdf_name)
    # joint_states = [0, 0, 0, 0, 0, 0]
    # robot.show(cfg=dict(zip(joint_names, joint_states)))

    arm_controller = ArmController("/dev/ttyUSB5")
    
    def get_leader_arm_state():
        pos, _, _, _ = arm_controller.get_pos()
        joint_states = -pos[:6] # z-axis in urdf are opposite to the urdf
        GRIPPER_READ_MIN = 0.0303
        GRIPPER_READ_MAX = 0.0383
        gripper_state = clip((pos[6] - GRIPPER_READ_MIN) / (GRIPPER_READ_MAX - GRIPPER_READ_MIN), 0, 1)
        return joint_states, gripper_state
    
    control_T = 0.01
    while True:
        joint_states, gripper_state = get_leader_arm_state()
        
        if gripper_state > 0.7:
            break
        
        print(joint_states)
        print(gripper_state)
        
        print("waiting for open gripper...")
        time.sleep(control_T)
        
    
    control_T = 0.01
    while True:
        joint_states, gripper_state = get_leader_arm_state()
        
        if gripper_state < 0.3:
            break
        
        print(joint_states)
        print(gripper_state)
        
        print("waiting for close gripper...")
        time.sleep(control_T)
    
    Tbaseiniteef = mr.FKinSpace(M, Slist, joint_states)
    Tiniteefbase = np.linalg.inv(Tbaseiniteef)
    
    control_T = 0.01
    while True:
        joint_states, gripper_state = get_leader_arm_state()
        
        print(joint_states)
        print(gripper_state)

        # fk = robot.link_fk(cfg=dict(zip(joint_names, joint_states)))
        # print(fk[robot.link_map['link_eef']])
        # print(mr.FKinSpace(M, Slist, joint_states))
        # robot.show(cfg=dict(zip(joint_names, joint_states)))

        Tbaseeef = mr.FKinSpace(M, Slist, joint_states)
        
        # # Disable matplotlib installed by system
        # # pip install -U matplotlib
        # # sudo mv /usr/lib/python3/dist-packages/matplotlib-3.5.1-nspkg.pth /usr/lib/python3/dist-packages/matplotlib-3.5.1-nspkg.pth.bak
        # # sudo mv /usr/lib/python3/dist-packages/matplotlib /usr/lib/python3/dist-packages/matplotlib.bak
        # # sudo mv /usr/lib/python3/dist-packages/mpl_toolkits /usr/lib/python3/dist-packages/mpl_toolkits.bak
        # # pip uninstall PyQt5 # use system installed PyQt5
        # # pip uninstall opencv-python # use system installed opencv-python
        # fig = plt.figure(figsize=(5, 5))
        # ax = fig.add_subplot(111, projection="3d")
        # ax.set_xlim((-0.5, 0.5))
        # ax.set_ylim((-0.5, 0.5))
        # ax.set_zlim((-0.5, 0.5))
        # ax.set_xlabel("X")
        # ax.set_ylabel("Y")
        # ax.set_zlabel("Z")
        # Frame(np.eye(4), label="s", s=0.1).add_frame(ax)
        # Frame(fk[robot.link_map['link1']], s=0.1).add_frame(ax)
        # Frame(fk[robot.link_map['link2']], label="link2", s=0.1).add_frame(ax)
        # Frame(fk[robot.link_map['link3']], label="link3", s=0.1).add_frame(ax)
        # Frame(fk[robot.link_map['link6']], label="link6", s=0.1).add_frame(ax)
        # # Frame(fk[robot.link_map['link_eef']], label="link_eef", s=0.1).add_frame(ax)
        # Frame(Tbaseeef, label="link_eef", s=0.1).add_frame(ax)
        # plt.show()
        
        Tiniteefeef = Tiniteefbase @ Tbaseeef
        
        Tiniteefeef[:3, 3] *= 2.0
        
        Tsgoal = Tsinitgoal @ Tiniteefeef
        
        print(Tsgoal)
        
        goal_gripper = gripper_state * GRIPPER_MAX

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