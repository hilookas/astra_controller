import rclpy
import rclpy.node
import rclpy.publisher

import geometry_msgs.msg
from astra_teleop.process import get_process
from pytransform3d import transformations as pt
import numpy as np
from pprint import pprint

def main(args=None):
    rclpy.init(args=args)
    
    node = rclpy.node.Node("teleop_node")

    pub_cam = node.create_publisher(geometry_msgs.msg.PoseStamped, "cam_pose", 10)
    pub = node.create_publisher(geometry_msgs.msg.PoseStamped, "goal_pose", 10)
    # pub1 = node.create_publisher(geometry_msgs.msg.PoseStamped, "goal_pose1", 10)
    # pub2 = node.create_publisher(geometry_msgs.msg.PoseStamped, "goal_pose2", 10)

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

    process = get_process(device="/dev/video0", calibration_directory="./calibration_images", debug=False)
    while True:
        tag2cam_left, tag2cam_right = process()
        pprint(tag2cam_left)
        pprint(tag2cam_right)
        cb(tag2cam_right)

    rclpy.spin(node) # will never go here

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()