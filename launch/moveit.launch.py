from moveit_configs_utils import MoveItConfigsBuilder

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from srdfdom.srdf import SRDF

from moveit_configs_utils.launch_utils import (
    add_debuggable_node,
    DeclareBooleanLaunchArg,
)

def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("astra_description", package_name="astra_moveit_config").to_moveit_configs()
    
    """
    Launches a self contained demo

    Includes
     * static_virtual_joint_tfs
     * robot_state_publisher
     * move_group
     * moveit_rviz
     * warehouse_db (optional)
     * ros2_control_node + controller spawners
    """
    ld = LaunchDescription()
    ld.add_action(
        DeclareBooleanLaunchArg(
            "debug",
            default_value=False,
            description="By default, we are not in debug mode",
        )
    )

    # broadcast static tf by including virtual_joints launch
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(moveit_config.package_path / "launch/static_virtual_joint_tfs.launch.py")),
        )
    )

    # Given the published joint states, publish tf for the robot links
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(moveit_config.package_path / "launch/rsp.launch.py")
            ),
        )
    )

    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(moveit_config.package_path / "launch/move_group.launch.py")
            ),
        )
    )

    # Run Rviz and load the default config to see the state of the move_group node
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(moveit_config.package_path / "launch/moveit_rviz.launch.py")
            ),
        )
    )

    ld.add_action(
        Node(
            package="astra_controller",
            executable="moveit_relay_node",
        )
    )

    # ld.add_action(
    #     Node(
    #         package="astra_controller",
    #         executable="dry_run_node",
    #         parameters=[
    #             {
    #                 "actively_send_joint_state": True,
    #             },
    #         ],
    #     )
    # )

    ld.add_action(
        Node(
            package="astra_controller",
            executable="arm_node",
        )
    )

    ld.add_action(
        Node(
            package="astra_controller",
            executable="lift_node",
        )
    )

    return ld
