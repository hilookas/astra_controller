import rclpy
import rclpy.node
import rclpy.qos
import rclpy.action

import nav_msgs.msg
import geometry_msgs.msg
import std_msgs.msg
import tf2_ros
import math

from .base_controller import BaseController

from pytransform3d import rotations as pr

def main(args=None):
    rclpy.init(args=args)

    node = rclpy.node.Node('base_node')

    logger = node.get_logger()

    node.declare_parameter('device', 'can0')

    device = node.get_parameter('device').value

    base_controller = BaseController(device)

    pos_x = 0
    pos_y = 0
    pos_theta = 0

    odom_publisher = node.create_publisher(nav_msgs.msg.Odometry, "odom", 10)
    tf_broadcaster = tf2_ros.TransformBroadcaster(node)

    def cb(linear_vel, angular_vel, dt):
        nonlocal pos_x, pos_y, pos_theta

        d_x = linear_vel * math.cos(pos_theta) * dt
        d_y = linear_vel * math.sin(pos_theta) * dt
        d_theta = angular_vel * dt

        pos_x += d_x
        pos_y += d_y
        pos_theta += d_theta
        quat = pr.quaternion_from_angle(2, pos_theta)


        odom_msg = nav_msgs.msg.Odometry()
        odom_msg.header.stamp = node.get_clock().now().to_msg()
        odom_msg.header.frame_id = "odom" # odom is fixed to map, base_link is reletive to odom
        odom_msg.child_frame_id = "base_link"

        odom_msg.pose.pose.position.x = pos_x
        odom_msg.pose.pose.position.y = pos_y
        odom_msg.pose.pose.position.z = 0.0
        odom_msg.pose.pose.orientation.w = quat[0]
        odom_msg.pose.pose.orientation.x = quat[1]
        odom_msg.pose.pose.orientation.y = quat[2]
        odom_msg.pose.pose.orientation.z = quat[3]

        odom_msg.twist.twist.linear.x = linear_vel
        odom_msg.twist.twist.linear.y = 0.0
        odom_msg.twist.twist.angular.z = angular_vel

        odom_publisher.publish(odom_msg)


        tf_msg = geometry_msgs.msg.TransformStamped()
        tf_msg.header.stamp = node.get_clock().now().to_msg()
        tf_msg.header.frame_id = "odom"
        tf_msg.child_frame_id = "base_link"

        tf_msg.transform.translation.x = odom_msg.pose.pose.position.x
        tf_msg.transform.translation.y = odom_msg.pose.pose.position.y
        tf_msg.transform.translation.z = odom_msg.pose.pose.position.z
        tf_msg.transform.rotation = odom_msg.pose.pose.orientation

        tf_broadcaster.sendTransform(tf_msg)
    base_controller.state_cb = cb

    def cb(msg: geometry_msgs.msg.Twist):
        base_controller.set_vel(msg.linear.x, msg.angular.z)
    node.create_subscription(geometry_msgs.msg.Twist, 'cmd_vel', cb, rclpy.qos.qos_profile_sensor_data)
    
    debug_publishers = {}
    debug_publishers["curpos_left"] = node.create_publisher(std_msgs.msg.Float32, "base/debug/curpos_left", 10)
    debug_publishers["curpos_right"] = node.create_publisher(std_msgs.msg.Float32, "base/debug/curpos_right", 10)
    debug_publishers["setpos_left"] = node.create_publisher(std_msgs.msg.Float32, "base/debug/setpos_left", 10)
    debug_publishers["setpos_right"] = node.create_publisher(std_msgs.msg.Float32, "base/debug/setpos_right", 10)
    def debug_cb(name, value):
        assert name in debug_publishers
        msg = std_msgs.msg.Float32()
        msg.data = value
        debug_publishers[name].publish(msg)
    base_controller.debug_cb = debug_cb

    try:
        rclpy.spin(node)
    except KeyboardInterrupt as err:
        base_controller.stop()
        raise err

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
