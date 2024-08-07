from typing import Any, List, Tuple, Union

import modern_robotics as mr
import numpy as np
from mr_urdf_loader import loadURDF

import math

import geometry_msgs.msg
import rclpy.node
from pytransform3d import transformations as pt
from rclpy.publisher import Publisher
import sensor_msgs.msg
import numpy as np
import rclpy.qos

from pprint import pprint

import logging

logger = logging.getLogger(__name__)
# logging.basicConfig(level=logging.DEBUG)

np.set_printoptions(precision=4, suppress=True)

rclpy.init()
node = rclpy.node.Node("arm_controller")

joint_state_publisher = node.create_publisher(sensor_msgs.msg.JointState, "/joint_states", 10)

# Source: interbotix_ros_toolboxes/interbotix_xs_toolbox/interbotix_xs_modules/interbotix_xs_modules/xs_robot/arm.py
class AstraArmInterface:
    def __init__(self,  
        initial_guesses: List[List[float]] | None = None,
    ):
        self.urdf_name = "/home/rosdev/ros2_ws/src/astra_description/urdf/astra_description_rel.urdf"

        self.num_joints = 6
        self.joint_lower_limits = [0, -math.pi/2, -math.pi/2, -math.pi/2, -math.pi/2, -math.pi/2]
        self.joint_upper_limits = [1, math.pi/2, math.pi/2, math.pi/2, math.pi/2, math.pi/2]
        self.joint_velocity_limits = [10, 10, 10, 10, 10, 10]
        
        self.moving_time = 1 / 50 # 50Hz TODO set
        self.joint_commands = [0, 0, 0, 0, 0, 0]
        self.group_name = "astra_right"

        self.M, self.Slist, self.Blist, self.Mlist, self.Glist, self.actuated_joints_num = loadURDF(
            self.urdf_name, 
            eef_link_name="link_ree", 
            actuated_joint_names=['joint_r1', 'joint_r2', 'joint_r3', 'joint_r4', 'joint_r5', 'joint_r6']
        )

        if initial_guesses is None:
            self.initial_guesses = [[0.0] * self.num_joints for _ in range(1)]
            # self.initial_guesses[1][0] = np.deg2rad(-120)
            # self.initial_guesses[2][0] = np.deg2rad(120)
        else:
            self.initial_guesses = initial_guesses

    def _check_joint_limits(self, positions: List[float]) -> bool:
        """
        Ensure the desired arm group's joint positions are within their limits.

        :param positions: the positions [rad] to check
        :return: `True` if all positions are within limits; `False` otherwise
        """
        logger.debug(f'Checking joint limits for {positions=}')
        theta_list = [int(elem * 1000) / 1000.0 for elem in positions]
        speed_list = [
            abs(goal - current) / float(self.moving_time)
            for goal, current in zip(theta_list, self.joint_commands)
        ]
        # check position and velocity limits
        for x in range(self.num_joints):
            if not (
                self.joint_lower_limits[x]
                <= theta_list[x]
                <= self.joint_upper_limits[x]
            ):
                return False
            if speed_list[x] > self.joint_velocity_limits[x]:
                return False
        return True
    
    def _publish_commands(
        self,
        positions: List[float],
    ) -> None:
        """
        Publish joint positions and block if necessary.

        :param positions: desired joint positions
        :param moving_time: (optional) duration in seconds that the robot should move
        :param accel_time: (optional) duration in seconds that that robot should spend
            accelerating/decelerating (must be less than or equal to half the moving_time)
        :param blocking: (optional) whether the function should wait to return control to the user
            until the robot finishes moving
        """
        logger.debug(f'Publishing {positions=}')
        self.joint_commands = list(positions)
        # joint_commands = JointGroupCommand(
        #     name=self.group_name, cmd=self.joint_commands
        # )
        # self.publish(joint_commands)
        
        msg = sensor_msgs.msg.JointState()
        msg.header.stamp = node.get_clock().now().to_msg()
        
        msg.name = [ "joint_r1", "joint_r2", "joint_r3", "joint_r4", "joint_r5", "joint_r6" ]
        msg.position = self.joint_commands
        print("publish")
        print(self.joint_commands)
        joint_state_publisher.publish(msg)

    def set_ee_pose_matrix(
        self,
        T_sd: np.ndarray,
    ) -> Tuple[Union[np.ndarray, Any, List[float]], bool]:
        """
        Command a desired end effector pose.

        :param T_sd: 4x4 Transformation Matrix representing the transform from the
            /<robot_name>/base_link frame to the /<robot_name>/ee_gripper_link frame
        :param custom_guess: (optional) list of joint positions with which to seed the IK solver
        :param execute: (optional) if `True`, this moves the physical robot after planning;
            otherwise, only planning is done
        :param moving_time: (optional) duration in seconds that the robot should move
        :param accel_time: (optional) duration in seconds that that robot should spend
            accelerating/decelerating (must be less than or equal to half the moving_time)
        :param blocking: (optional) whether the function should wait to return control to the user
            until the robot finishes moving
        :return: joint values needed to get the end effector to the desired pose
        :return: `True` if a valid solution was found; `False` otherwise
        """
        logger.debug(f'Setting ee_pose to matrix=\n{T_sd}')
        initial_guesses = self.initial_guesses

        for guess in initial_guesses:
            theta_list, success = mr.IKinSpace(
                Slist=self.Slist,
                M=self.M,
                T=T_sd,
                thetalist0=guess,
                eomg=0.001,
                ev=0.001,
            )
            
            if not success:
                logger.warn('failed guess')
                continue

            # # Check to make sure a solution was found and that no joint limits were violated
            # solution_found = self._check_joint_limits(theta_list)
            # if not solution_found:
            #     logger.warn('failed guess')
            #     continue
            
            self._publish_commands(
                theta_list
            )
            return theta_list, True

        logger.warn('No valid pose could be found. Will not execute')
        return theta_list, False

astra_arm = AstraArmInterface()

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
    astra_arm.set_ee_pose_matrix(pt.transform_from_pq(pq))

pub = node.create_subscription(geometry_msgs.msg.PoseStamped, "/goal_pose", cb, rclpy.qos.qos_profile_sensor_data)

rclpy.spin(node)

# Destroy the node explicitly
# (optional - otherwise it will be done automatically
# when the garbage collector destroys the node object)
node.destroy_node()
rclpy.shutdown()
