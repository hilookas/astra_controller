import rclpy
import rclpy.node
import rclpy.qos

import geometry_msgs.msg
import astra_controller_interfaces.msg

from typing import Any, List, Tuple, Union

import modern_robotics as mr
import numpy as np
from mr_urdf_loader import loadURDF
from pytransform3d import transformations as pt

import logging

logger = logging.getLogger(__name__)
# logging.basicConfig(level=logging.DEBUG)

np.set_printoptions(precision=4, suppress=True)

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.node.Node("ik_node")

    arm_joint_command_publisher = node.create_publisher(astra_controller_interfaces.msg.JointGroupCommand, "/arm_joint_command", 10)

    lift_joint_command_publisher = node.create_publisher(astra_controller_interfaces.msg.JointGroupCommand, "/lift_joint_command", 10)

    # Ref: interbotix_ros_toolboxes/interbotix_xs_toolbox/interbotix_xs_modules/interbotix_xs_modules/xs_robot/arm.py
    urdf_name = "/home/rosdev/ros2_ws/src/astra_description/urdf/astra_description_rel.urdf"
    
    eef_link_name="link_ree"
    actuated_joint_names=['joint_r1', 'joint_r2', 'joint_r3', 'joint_r4', 'joint_r5', 'joint_r6']

    M, Slist, Blist, Mlist, Glist, robot = loadURDF(
        urdf_name, 
        eef_link_name=eef_link_name, 
        actuated_joint_names=actuated_joint_names
    )
    
    joint_limit_lower = []
    joint_limit_upper = []
    for joint_name in actuated_joint_names:
        joint = robot.joint_map[joint_name]
        joint_limit_lower.append(joint.limit.lower)
        joint_limit_upper.append(joint.limit.upper)

    def set_ee_pose_matrix(
        T_sd: np.ndarray,
    ) -> Tuple[Union[np.ndarray, Any, List[float]], bool]:
        """
        Command a desired end effector pose.

        :param T_sd: 4x4 Transformation Matrix representing the transform from the
            /<robot_name>/base_link frame to the /<robot_name>/ee_gripper_link frame
        :return: joint values needed to get the end effector to the desired pose
        :return: `True` if a valid solution was found; `False` otherwise
        """
        logger.debug(f'Setting ee_pose to matrix=\n{T_sd}')

        theta_list, success = mr.IKinSpace(
            Slist=Slist,
            M=M,
            T=T_sd,
            thetalist0=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            eomg=0.001,
            ev=0.001,
        )
        
        if not success:
            logger.warn('failed guess')
            logger.warn('No valid pose could be found. Will not execute')
            return theta_list, False

        # Check to make sure a solution was found and that no joint limits were violated
        if not ((joint_limit_lower <= theta_list) & (theta_list <= joint_limit_upper)).all():
            logger.warn('guess over range')
            logger.warn('No valid pose could be found. Will not execute')
            return theta_list, False

        msg = astra_controller_interfaces.msg.JointGroupCommand(
            cmd=list(theta_list[1:])
        )
        arm_joint_command_publisher.publish(msg)

        msg = astra_controller_interfaces.msg.JointGroupCommand(
            cmd=list(theta_list[:1])
        )
        lift_joint_command_publisher.publish(msg)

        return theta_list, True

    def cb(msg: geometry_msgs.msg.PoseStamped):
        pq = np.array([
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z,
            msg.pose.orientation.w,
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z
        ])
        set_ee_pose_matrix(pt.transform_from_pq(pq))
    node.create_subscription(geometry_msgs.msg.PoseStamped, "/goal_pose", cb, rclpy.qos.qos_profile_sensor_data)

    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
