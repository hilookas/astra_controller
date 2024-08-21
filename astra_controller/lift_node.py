import rclpy
import rclpy.node
import rclpy.qos
import rclpy.action

import sensor_msgs.msg
import astra_controller_interfaces.msg
import astra_controller_interfaces.srv

from .lift_controller import LiftController

def main(args=None):
    rclpy.init(args=args)

    node = rclpy.node.Node('lift_node')

    logger = node.get_logger()
    
    node.declare_parameter('device', '/dev/tty_puppet_lift_right')
    node.declare_parameter('side', 'right')

    device = node.get_parameter('device').value
    side = node.get_parameter('side').value
    
    if side not in ['left', 'right']:
        raise Exception("Unknown side")
    
    side_config = {
        "left": {
            "joint_names": [
                "joint_l1",
            ],
        },
        "right": {
            "joint_names": [
                "joint_r1",
            ],
        },
    }

    lift_controller = LiftController(device)

    joint_state_publisher = node.create_publisher(sensor_msgs.msg.JointState, "joint_states", 10)
    def cb(position, velocity, effort, this_time):
        msg = sensor_msgs.msg.JointState()
        msg.header.stamp = node.get_clock().now().to_msg()
        
        msg.name = side_config[side]["joint_names"]
        msg.position = [ 
            float(position),
        ]
        msg.velocity = [ 
            float(velocity),
        ]
        msg.effort = [ 
            float(effort),
        ]

        joint_state_publisher.publish(msg)
    lift_controller.state_cb = cb
    
    def cb(msg: astra_controller_interfaces.msg.JointGroupCommand):
        lift_controller.set_pos(msg.cmd[0])
    node.create_subscription(astra_controller_interfaces.msg.JointGroupCommand, 'joint_command', cb, rclpy.qos.qos_profile_sensor_data)

    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
