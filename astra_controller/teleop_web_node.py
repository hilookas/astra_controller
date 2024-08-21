from pprint import pprint
import rclpy
import rclpy.node
import rclpy.publisher

import geometry_msgs.msg
from astra_teleop_web.webserver import WebServer
from pytransform3d import transformations as pt
import numpy as np
import threading

import PIL.Image
import io
import sensor_msgs.msg

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.node.Node("teleop_node")

    webserver = WebServer()
    
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

    pub_cam = node.create_publisher(geometry_msgs.msg.PoseStamped, f"cam_pose", 10)
    pub = node.create_publisher(geometry_msgs.msg.PoseStamped, f"goal_pose", 10)
    # pub1 = node.create_publisher(geometry_msgs.msg.PoseStamped, f"goal_pose1", 10)
    # pub2 = node.create_publisher(geometry_msgs.msg.PoseStamped, f"goal_pose2", 10)

    Tcamgoal_last = None

    def cb(
        Tcamgoal, 
        # Tcamgoal1, Tcamgoal2
    ):
        # 将摄像头坐标系从base link坐标系 移动+旋转到正确位置
        Tscam = np.array([
            [0, 0, -1, 1.5], 
            [1, 0, 0, -0.5], 
            [0, -1, 0, 0.5], 
            [0, 0, 0, 1], 
        ])
        pub_T(pub_cam, Tscam)

        nonlocal Tcamgoal_last
        if Tcamgoal_last is None:
            Tcamgoal_last = Tcamgoal
        low_pass_coff = 0.4
        Tcamgoal = pt.transform_from_pq(pt.pq_slerp(
            pt.pq_from_transform(Tcamgoal_last),
            pt.pq_from_transform(Tcamgoal),
            low_pass_coff
        ))
        Tcamgoal_last = Tcamgoal
        
        Tsgoal = Tscam @ Tcamgoal
        pub_T(pub, Tsgoal)
        
        # pub_T(pub1, Tscam @ Tcamgoal1)
        # pub_T(pub2, Tscam @ Tcamgoal2)
    
    webserver.left_hand_cb = cb
    
    
    def get_cb(name):
        def cb(msg):
            image = PIL.Image.open(io.BytesIO(msg.data))
            if name == "head":
                image = image.resize((1280, 720))
            else:
                image = image.resize((640, 360))
            try:
                getattr(webserver, f"track_{name}").feed(image)
            except:
                pass
        return cb
    node.create_subscription(
        sensor_msgs.msg.CompressedImage, 'cam_head/image_raw/compressed', get_cb("head"), rclpy.qos.qos_profile_sensor_data 
    )
    node.create_subscription(
        sensor_msgs.msg.CompressedImage, 'left/cam_wrist/image_raw/compressed', get_cb("wrist_left"), rclpy.qos.qos_profile_sensor_data 
    )
    node.create_subscription(
        sensor_msgs.msg.CompressedImage, 'right/cam_wrist/image_raw/compressed', get_cb("wrist_right"), rclpy.qos.qos_profile_sensor_data 
    )
    # rclpy.qos.qos_profile_sensor_data: best effort reliability and a smaller queue size
    # see: https://docs.ros.org/en/rolling/Concepts/Intermediate/About-Quality-of-Service-Settings.html
    # see: https://blog.csdn.net/qq_38649880/article/details/105908598


    cmd_vel_publisher = node.create_publisher(geometry_msgs.msg.Twist, 'cmd_vel', 10)
    
    def cb(pedal_real_values):
        non_sensetive_area = 0.1
        cliped_pedal_real_values = np.clip((np.array(pedal_real_values) - 0.5) / (0.5 - non_sensetive_area) * 0.5 + 0.5, 0, 1)
        pedal_names = ["linear-neg", "linear-pos", "angular-neg", "angular-pos", "mode-select", "left-gripper", "right-gripper"]
        values = dict(zip(pedal_names, cliped_pedal_real_values))
        LINEAR_VEL_MAX = 1
        ANGULAR_VEL_MAX = 1
        linear_vel = (values["linear-pos"] - values["linear-neg"]) * LINEAR_VEL_MAX
        angular_vel = (values["angular-pos"] - values["angular-neg"]) * ANGULAR_VEL_MAX
        
        msg = geometry_msgs.msg.Twist()

        msg.linear.x = linear_vel
        msg.angular.z = angular_vel

        cmd_vel_publisher.publish(msg)
        
    webserver.pedal_cb = cb

    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()