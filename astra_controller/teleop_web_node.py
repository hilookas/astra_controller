import asyncio
import json
import threading
import time
import rclpy
import rclpy.node
import rclpy.publisher

from astra_teleop_web.webserver import WebServer
from pytransform3d import transformations as pt
import numpy as np

import sensor_msgs.msg

import rclpy
import rclpy.node
import rclpy.qos

import std_msgs.msg
import geometry_msgs.msg
import astra_controller_interfaces.msg

import numpy as np
from pytransform3d import transformations as pt
from pytransform3d import rotations as pr

from tf2_ros.transform_listener import TransformListener
from tf2_ros.buffer import Buffer

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

    webserver = WebServer()
    
    tf_buffer = Buffer()
    tf_listener = TransformListener(tf_buffer, node)
    
    arm_enabled = False
    lift_distance = 0.8
    
    GRIPPER_MAX = 0.055
    
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
    
    joint_states = {
        "joint_l1": 0.0, "joint_l2": 0.0, "joint_l3": 0.0, "joint_l4": 0.0, "joint_l5": 0.0, "joint_l6": 0.0, "joint_l7r": 0.0,
        "joint_r1": 0.0, "joint_r2": 0.0, "joint_r3": 0.0, "joint_r4": 0.0, "joint_r5": 0.0, "joint_r6": 0.0, "joint_r7r": 0.0,
    }

    def cb(msg: sensor_msgs.msg.JointState):
        joint_states.update(dict(zip(msg.name, msg.position)))
    node.create_subscription(sensor_msgs.msg.JointState, "joint_states", cb, rclpy.qos.qos_profile_sensor_data)

    def get_cb(side):
        pub_cam = node.create_publisher(geometry_msgs.msg.PoseStamped, f"{side}/cam_pose", 10)
        pub_goal = node.create_publisher(geometry_msgs.msg.PoseStamped, f"{side}/goal_pose", 10)
        pub_goal_inactive = node.create_publisher(geometry_msgs.msg.PoseStamped, f"{side}/goal_pose_inactive", 10)
        # pub1 = node.create_publisher(geometry_msgs.msg.PoseStamped, f"{side}/goal_pose1", 10)
        # pub2 = node.create_publisher(geometry_msgs.msg.PoseStamped, f"{side}/goal_pose2", 10)
        
        if side == "left":
            Tscam = np.array([
                [0, 0, -1, 1.0], 
                [1, 0, 0, 0.5], 
                [0, -1, 0, lift_distance], 
                [0, 0, 0, 1], 
            ])
        elif side == "right":
            Tscam = np.array([
                [0, 0, -1, 1.0], 
                [1, 0, 0, -0.5], 
                [0, -1, 0, lift_distance], 
                [0, 0, 0, 1], 
            ])

        Tcamgoal_last = None
        
        def reset_Tscam():
            nonlocal Tscam
            Tsgoal_msg: geometry_msgs.msg.TransformStamped = tf_buffer.lookup_transform('base_link', 'link_ree_teleop' if side == "right" else 'link_lee_teleop', rclpy.time.Time())
            Tsgoal = pt.transform_from_pq(np.array(pq_from_ros_transform(Tsgoal_msg.transform)))

            if Tcamgoal_last is None:
                raise Exception(f"Connect your {side} hand capture first!")
            Tcamgoal = Tcamgoal_last
            Tscam = Tsgoal @ np.linalg.inv(Tcamgoal)
            logger.info(f"{side} Tscam reset")
            logger.info(str(Tscam))

        def cb(
            Tcamgoal, 
            # Tcamgoal1, Tcamgoal2
        ):
            # 将摄像头坐标系从base link坐标系 移动+旋转到正确位置
            pub_T(pub_cam, Tscam)

            nonlocal Tcamgoal_last
            if Tcamgoal_last is None:
                Tcamgoal_last = Tcamgoal

            # low_pass_coff = 0.1
            # Tcamgoal = pt.transform_from_pq(pt.pq_slerp(
            #     pt.pq_from_transform(Tcamgoal_last),
            #     pt.pq_from_transform(Tcamgoal),
            #     low_pass_coff
            # ))

            # # trust for sensor read (in this case, opencv on smartphone)
            # p_low_pass_coff = 0.99
            # q_low_pass_coff = 0.80
            # pq_camgoal_last = pt.pq_from_transform(Tcamgoal_last)
            # pq_camgoal = pt.pq_from_transform(Tcamgoal)
            # p = pq_camgoal_last[:3] * (1 - p_low_pass_coff) + pq_camgoal[:3] * p_low_pass_coff
            # q = pr.quaternion_slerp(pq_camgoal_last[3:], pq_camgoal[3:], q_low_pass_coff)
            # Tcamgoal = pt.transform_from_pq(np.concatenate([p, q]))

            Tcamgoal_last = Tcamgoal
            
            Tsgoal = Tscam @ Tcamgoal
            pub_T(pub_goal_inactive, Tsgoal)
            # pub_T(pub1, Tscam @ Tcamgoal1)
            # pub_T(pub2, Tscam @ Tcamgoal2)
            if arm_enabled:
                pub_T(pub_goal, Tsgoal)
        
        arm_joint_command_publisher = node.create_publisher(astra_controller_interfaces.msg.JointGroupCommand, f"{side}/arm/joint_command", 10)
        lift_joint_command_publisher = node.create_publisher(astra_controller_interfaces.msg.JointGroupCommand, f"{side}/lift/joint_command", 10)
        arm_gripper_joint_command_publisher = node.create_publisher(astra_controller_interfaces.msg.JointGroupCommand, f"{side}/arm/gripper_joint_command", 10)

        def command_arm(command_joint_states):
            arm_joint_command_publisher.publish(astra_controller_interfaces.msg.JointGroupCommand(
                cmd=list(command_joint_states[1:6])
            ))

            lift_joint_command_publisher.publish(astra_controller_interfaces.msg.JointGroupCommand(
                cmd=list(command_joint_states[:1])
            ))

            arm_gripper_joint_command_publisher.publish(astra_controller_interfaces.msg.JointGroupCommand(
                cmd=list(command_joint_states[6:7])
            ))
                
        async def reset_arm():
            lift_distance = 0.8
            if side == "left":
                joint_names = ["joint_l1", "joint_l2", "joint_l3", "joint_l4", "joint_l5", "joint_l6", "joint_l7r", ]
                initial_joint_states = [lift_distance, 0.785, -0.785, 0, 0, 0, GRIPPER_MAX]
            else:
                joint_names = ["joint_r1", "joint_r2", "joint_r3", "joint_r4", "joint_r5", "joint_r6", "joint_r7r", ]
                initial_joint_states = [lift_distance, -0.785, 0.785, 0, 0, 0, GRIPPER_MAX]

            while True:
                current_joint_states = [joint_states[key] for key in joint_names]
                
                dist = np.linalg.norm(np.array(current_joint_states) - initial_joint_states)
                
                if dist < 0.1:
                    break
                
                command_arm(initial_joint_states)
                await asyncio.sleep(0.1)
              
            for i in range(5):
                command_arm(initial_joint_states)
                await asyncio.sleep(0.1)
        
        def set_gripper(gripper_open):
            arm_gripper_joint_command_publisher.publish(astra_controller_interfaces.msg.JointGroupCommand(
                cmd=list([gripper_open])
            ))
        
        def Tscam_update_lift_distance(lift_distance_change):
            Tscam[2,3] += lift_distance_change
        return cb, reset_Tscam, reset_arm, Tscam_update_lift_distance, set_gripper
    
    webserver.right_hand_cb, reset_Tscam_right, reset_arm_right, Tscam_update_lift_distance_right, set_gripper_right = get_cb("right")
    webserver.left_hand_cb, reset_Tscam_left, reset_arm_left, Tscam_update_lift_distance_left, set_gripper_left = get_cb("left")
    
    def get_cb(name):
        def cb(msg: sensor_msgs.msg.Image):
            assert msg.encoding == "rgb8"
            image = np.asarray(msg.data).reshape(msg.height, msg.width, 3)
            if name == "head":
                assert msg.height == 360 and msg.width == 640
                # image = cv2.resize(image, (1280, 720))
            else:
                assert msg.height == 360 and msg.width == 640
                # image = cv2.resize(image, (640, 360))
            try:
                getattr(webserver, f"track_{name}").feed(image)
            except:
                pass
        return cb
    node.create_subscription(
        sensor_msgs.msg.Image, 'cam_head/image_raw', get_cb("head"), qos_profile_sensor_data_reliable
    )
    node.create_subscription(
        sensor_msgs.msg.Image, 'left/cam_wrist/image_raw', get_cb("wrist_left"), qos_profile_sensor_data_reliable
    )
    node.create_subscription(
        sensor_msgs.msg.Image, 'right/cam_wrist/image_raw', get_cb("wrist_right"), qos_profile_sensor_data_reliable
    )
    # rclpy.qos.qos_profile_sensor_data: best effort reliability and a smaller queue size
    # see: https://docs.ros.org/en/rolling/Concepts/Intermediate/About-Quality-of-Service-Settings.html
    # see: https://blog.csdn.net/qq_38649880/article/details/105908598


    cmd_vel_publisher = node.create_publisher(geometry_msgs.msg.Twist, 'cmd_vel', 10)
    
    last_t = None
    def cb(pedal_real_values):
        nonlocal lift_distance, last_t

        non_sensetive_area = 0.1
        cliped_pedal_real_values = np.clip((np.array(pedal_real_values) - 0.5) / (0.5 - non_sensetive_area) * 0.5 + 0.5, 0, 1)
        pedal_names = ["angular-pos", "angular-neg", "linear-neg", "linear-pos"]
        pedal_names_arm_enabled = ["left-gripper", "lift-neg", "lift-pos", "right-gripper"]
        if arm_enabled:
            values = dict(zip(pedal_names_arm_enabled, cliped_pedal_real_values))

            LIFT_VEL_MAX = 0.5
            lift_vel = (values["lift-pos"] - values["lift-neg"]) * LIFT_VEL_MAX

            this_t = time.perf_counter()
            if last_t is None:
                last_t = this_t
            lift_distance_change = (this_t - last_t) * lift_vel
            LIFT_DISTANCE_MIN = 0.7
            LIFT_DISTANCE_MAX = 1.2
            if lift_distance + lift_distance_change < LIFT_DISTANCE_MIN or lift_distance + lift_distance_change > LIFT_DISTANCE_MAX:
                logger.warn("lift over limit")
            elif lift_distance_change:
                Tscam_update_lift_distance_left(lift_distance_change)
                Tscam_update_lift_distance_right(lift_distance_change)
                lift_distance += lift_distance_change
                logger.info(str(lift_distance))
            last_t = this_t
            
            left_gripper_pos = (1 - values["left-gripper"]) * GRIPPER_MAX
            right_gripper_pos = (1 - values["right-gripper"]) * GRIPPER_MAX
            
            set_gripper_right(right_gripper_pos)
            set_gripper_left(left_gripper_pos)
        else:
            values = dict(zip(pedal_names, cliped_pedal_real_values))

            LINEAR_VEL_MAX = 1
            ANGULAR_VEL_MAX = 1
            linear_vel = (values["linear-pos"] - values["linear-neg"]) * LINEAR_VEL_MAX
            angular_vel = (values["angular-pos"] - values["angular-neg"]) * ANGULAR_VEL_MAX * (-1 if linear_vel < 0 else 1)
            
            msg = geometry_msgs.msg.Twist()

            msg.linear.x = linear_vel
            msg.angular.z = angular_vel

            cmd_vel_publisher.publish(msg)

            last_t = None
        
    webserver.pedal_cb = cb

        
    def disable_arm_teleop():
        nonlocal arm_enabled
        arm_enabled = False
    
    def enable_arm_teleop():
        nonlocal arm_enabled
        reset_Tscam_left()
        reset_Tscam_right()
        arm_enabled = True
    
    reset_publisher = node.create_publisher(std_msgs.msg.Bool, 'reset', 10)
    done_publisher = node.create_publisher(std_msgs.msg.Bool, 'done', 10)
    
    def datachannel_log(message):
        if webserver.datachannel is not None:
            webserver.datachannel.send(json.dumps(message))
    
    async def reset_arm():
        try: 
            disable_arm_teleop()
            await asyncio.gather(reset_arm_left(), reset_arm_right())
            enable_arm_teleop() # uncomment if you collect data
            reset_publisher.publish(std_msgs.msg.Bool(data=True))
            datachannel_log("Reset")
        except Exception as e:
            err_msg = f"{type(e).__name__}: {e.args}"
            datachannel_log(err_msg)
            logger.warn(str(e))
        
    def cb(control_type):
        logger.info(control_type)
        datachannel_log(f"Cmd: {control_type}")
        if control_type == "disable_arm_teleop":
            disable_arm_teleop()
        elif control_type == "enable_arm_teleop":
            enable_arm_teleop()
        elif control_type == "reset":
            threading.Thread(target=asyncio.run, args=(reset_arm(),), daemon=True).start()
        elif control_type == "done":
            done_publisher.publish(std_msgs.msg.Bool(data=True))
            datachannel_log("Done")
        
    webserver.control_cb = cb

    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()