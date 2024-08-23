import math
from pathlib import Path
import rclpy
import rclpy.node
import rclpy.qos

import geometry_msgs.msg
import astra_controller_interfaces.msg

from ament_index_python import get_package_share_directory

from typing import Any, List, Tuple, Union

import modern_robotics as mr
import numpy as np
from mr_urdf_loader import loadURDF
from pytransform3d import transformations as pt

import logging

logger = logging.getLogger(__name__)

np.set_printoptions(precision=4, suppress=True)

def main(args=None):
    rclpy.init(args=args)

    node = rclpy.node.Node("ik_node")
    
    node.declare_parameter('side', 'right')

    side = node.get_parameter('side').value
    
    if side not in ['left', 'right']:
        raise Exception("Unknown side")
    
    side_config = {
        "left": {
            "eef_link_name": "link_lee",
            "actuated_joint_names": ['joint_l1', 'joint_l2', 'joint_l3', 'joint_l4', 'joint_l5', 'joint_l6'],
        },
        "right": {
            "eef_link_name": "link_ree",
            "actuated_joint_names": ['joint_r1', 'joint_r2', 'joint_r3', 'joint_r4', 'joint_r5', 'joint_r6'],
        },
    }

    # Ref: interbotix_ros_toolboxes/interbotix_xs_toolbox/interbotix_xs_modules/interbotix_xs_modules/xs_robot/arm.py
    urdf_name = str(Path(get_package_share_directory("astra_description")) / "urdf" / "astra_description_rel.urdf")
    
    eef_link_name=side_config[side]["eef_link_name"]
    actuated_joint_names=side_config[side]["actuated_joint_names"]

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

    arm_joint_command_publisher = node.create_publisher(astra_controller_interfaces.msg.JointGroupCommand, "arm/joint_command", 10)
    lift_joint_command_publisher = node.create_publisher(astra_controller_interfaces.msg.JointGroupCommand, "lift/joint_command", 10)
        
    def pub_theta(theta_list):
        msg = astra_controller_interfaces.msg.JointGroupCommand(
            cmd=list(theta_list[1:])
        )
        arm_joint_command_publisher.publish(msg)

        msg = astra_controller_interfaces.msg.JointGroupCommand(
            cmd=list(theta_list[:1])
        )
        lift_joint_command_publisher.publish(msg)
        
    last_theta_list = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

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
        
        nonlocal last_theta_list
        
        for initial_guess in [
            last_theta_list,
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        ]:
            theta_list, success = mr.IKinSpace(
                Slist=Slist,
                M=M,
                T=T_sd,
                thetalist0=initial_guess,
                eomg=0.001,
                ev=0.001,
            )
            
            if not success:
                logger.warn('Failed guess. Maybe EEF is out of range.')
                continue
            
            before_clip = theta_list
            if not ((joint_limit_lower <= theta_list) & (theta_list <= joint_limit_upper)).all():
                for i in [1, 2, 3, 4, 5]:
                    theta_list[i] = math.fmod(math.fmod(theta_list[i] + math.pi, 2*math.pi) + 2*math.pi, 2*math.pi) - math.pi

            # Check to make sure a solution was found and that no joint limits were violated
            if not ((joint_limit_lower <= theta_list) & (theta_list <= joint_limit_upper)).all():
                logger.warn('Guess over range.')
                logger.warn('min: ' + str(joint_limit_lower))
                logger.warn('before clip' + str(before_clip))
                logger.warn('after clip' + str(theta_list))
                logger.warn('max: ' + str(joint_limit_upper))
                continue
            
            pub_theta(theta_list)

            last_theta_list = theta_list
            return theta_list, True
        
        logger.warn('No valid pose could be found. Will not execute')
        return theta_list, False

    def pq_from_ros_pose(msg: geometry_msgs.msg.Pose):
        return np.array([
            msg.position.x,
            msg.position.y,
            msg.position.z,
            msg.orientation.w,
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z
        ])

    def cb(msg: geometry_msgs.msg.PoseStamped):
        set_ee_pose_matrix(pt.transform_from_pq(pq_from_ros_pose(msg.pose)))
    node.create_subscription(geometry_msgs.msg.PoseStamped, "goal_pose", cb, rclpy.qos.qos_profile_sensor_data)

    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
