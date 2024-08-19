from pprint import pprint
import rclpy
import rclpy.node
import rclpy.publisher

import geometry_msgs.msg
from astra_teleop_web.webserver import WebServer, feed_webserver
from pytransform3d import transformations as pt
import numpy as np
import threading

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.node.Node("teleop_node")

    pub_cam = node.create_publisher(geometry_msgs.msg.PoseStamped, "/cam_pose", 10)
    pub = node.create_publisher(geometry_msgs.msg.PoseStamped, "/goal_pose", 10)
    # pub1 = node.create_publisher(geometry_msgs.msg.PoseStamped, "/goal_pose1", 10)
    # pub2 = node.create_publisher(geometry_msgs.msg.PoseStamped, "/goal_pose2", 10)

    def pub_T(pub: rclpy.publisher.Publisher, T):
        msg = geometry_msgs.msg.PoseStamped()
        msg.header.frame_id = 'base_link'
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

    webserver = WebServer()
    
    threading.Thread(target=feed_webserver, args=(webserver, "head"), daemon=True).start()
    threading.Thread(target=feed_webserver, args=(webserver, "wrist_left"), daemon=True).start()
    threading.Thread(target=feed_webserver, args=(webserver, "wrist_right"), daemon=True).start()
    
    webserver.left_hand_cb = cb
    # webserver.right_hand_cb = cb
    
    cmd_vel_publisher = node.create_publisher(geometry_msgs.msg.Twist, '/cmd_vel', 10)
        
    def cb(pedal_real_values):
        # print("pedal")
        non_sensetive_area = 0.1
        cliped_pedal_real_values = np.clip((np.array(pedal_real_values) - 0.5) / (0.5 - non_sensetive_area) * 0.5 + 0.5, 0, 1)
        pedal_names = ["linear-neg", "linear-pos", "angular-neg", "angular-pos", "mode-select", "left-gripper", "right-gripper"]
        # pprint(cliped_pedal_real_values)
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

    rclpy.spin(node) # will never go here

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()