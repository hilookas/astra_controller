import rclpy
import rclpy.node
import rclpy.qos
import rclpy.action

import astra_controller_interfaces.msg
import control_msgs.action
import astra_controller_interfaces.srv

def main(args=None):
    rclpy.init(args=args)

    node = rclpy.node.Node('moveit_relay_node')

    arm_joint_command_publisher = node.create_publisher(astra_controller_interfaces.msg.JointCommand, "arm/joint_command", 10)

    gripper_joint_command_publisher = node.create_publisher(astra_controller_interfaces.msg.JointCommand, "arm/gripper_joint_command", 10)

    lift_joint_command_publisher = node.create_publisher(astra_controller_interfaces.msg.JointCommand, "lift/joint_command", 10)
    
    joint_pos = {
        "joint_r1": 0, 
        "joint_r2": 0, 
        "joint_r3": 0, 
        "joint_r4": 0, 
        "joint_r5": 0, 
        "joint_r6": 0, 
        "joint_r7l": 0, 
        "joint_r7r": 0
    }

    def execute_callback(goal_handle: rclpy.action.server.ServerGoalHandle):
        # https://docs.ros.org/en/foxy/Tutorials/Intermediate/Writing-an-Action-Server-Client/Py.html
        node.get_logger().info('Executing goal...')
        # print(goal_handle.request.trajectory.joint_names) # ['joint_r1', 'joint_r2', 'joint_r3', 'joint_r4', 'joint_r5', 'joint_r6']
        # print(goal_handle.request.trajectory.points[-1].positions) # trajectory_msgs.msg.JointTrajectoryPoint(positions=[0.1,0.2,...], velocities=...)

        for joint_name, position in zip(
            goal_handle.request.trajectory.joint_names, 
            goal_handle.request.trajectory.points[-1].positions
        ):
            joint_pos[joint_name] = position

        msg = astra_controller_interfaces.msg.JointCommand(
            name=[
                "joint_r2",
                "joint_r3",
                "joint_r4",
                "joint_r5",
                "joint_r6",
            ],
            position_cmd=[
                joint_pos["joint_r2"],
                joint_pos["joint_r3"],
                joint_pos["joint_r4"],
                joint_pos["joint_r5"],
                joint_pos["joint_r6"],
            ]
        )
        arm_joint_command_publisher.publish(msg)

        msg = astra_controller_interfaces.msg.JointCommand(
            name=[
                "joint_r1",
            ],
            position_cmd=[
                joint_pos["joint_r1"]
            ]
        )
        lift_joint_command_publisher.publish(msg)
        
        # ignore publish_feedback

        goal_handle.succeed()

        result = control_msgs.action.FollowJointTrajectory.Result()
        result.error_code = control_msgs.action.FollowJointTrajectory.Result.SUCCESSFUL
        result.error_string = "SUCC"
        return result
    rclpy.action.ActionServer(
        node,
        control_msgs.action.FollowJointTrajectory,
        '/astra_right_arm_controller/follow_joint_trajectory',
        execute_callback
    )

    def execute_callback(goal_handle: rclpy.action.server.ServerGoalHandle):
        # https://docs.ros.org/en/foxy/Tutorials/Intermediate/Writing-an-Action-Server-Client/Py.html
        node.get_logger().info('Executing goal...')
        # print(goal_handle.request.trajectory.joint_names) # ['joint_r1', 'joint_r2', 'joint_r3', 'joint_r4', 'joint_r5', 'joint_r6']
        # print(goal_handle.request.trajectory.points[-1].positions) # trajectory_msgs.msg.JointTrajectoryPoint(positions=[0.1,0.2,...], velocities=...)

        for joint_name, position in zip(
            goal_handle.request.trajectory.joint_names, 
            goal_handle.request.trajectory.points[-1].positions
        ):
            joint_pos[joint_name] = position

        msg = astra_controller_interfaces.msg.JointCommand(
            name=[
                "joint_r7r",
            ],
            position_cmd=[
                joint_pos["joint_r7r"]
            ]
        )
        gripper_joint_command_publisher.publish(msg)
        
        # ignore publish_feedback

        goal_handle.succeed()

        result = control_msgs.action.FollowJointTrajectory.Result()
        result.error_code = control_msgs.action.FollowJointTrajectory.Result.SUCCESSFUL
        result.error_string = "SUCC"
        return result
    rclpy.action.ActionServer(
        node,
        control_msgs.action.FollowJointTrajectory,
        '/astra_right_hand_controller/follow_joint_trajectory',
        execute_callback
    )

    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
