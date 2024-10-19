from pathlib import Path
from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    GroupAction
)
from launch.conditions import IfCondition
from launch.launch_description_sources import FrontendLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from srdfdom.srdf import SRDF

def generate_launch_description():
    ld = LaunchDescription()
    
    package_name = 'astra_controller'
    package_path = Path(get_package_share_directory(package_name))
    
    ld.add_action(
        DeclareLaunchArgument(
            "rviz_config",
            default_value=str(package_path / "config/default.rviz"),
        )
    )
    
    # Cameras
    ld.add_action(
        Node(
            package='usb_cam', executable='usb_cam_node_exe',
            namespace='cam_head',
            parameters=[{
                'video_device': '/dev/video_head',
                'pixel_format': 'mjpeg2rgb', # mjpeg2rgb to imdecode with usb_cam (usb_cam use ffmpeg decode which is more faster and less cpu usage)
                'image_width': 640,
                'image_height': 360,
                'framerate': 30.0,
            }],
            output={'both': 'log'},
        )
    )
    ld.add_action(
        Node(
            package='usb_cam', executable='usb_cam_node_exe',
            namespace='left/cam_wrist',
            parameters=[{
                'video_device': '/dev/video_wrist_left',
                'pixel_format': 'mjpeg2rgb',
                'image_width': 640,
                'image_height': 360,
                'framerate': 30.0,
            }],
            output={'both': 'log'},
        )
    )
    ld.add_action(
        Node(
            package='usb_cam', executable='usb_cam_node_exe',
            namespace='right/cam_wrist',
            parameters=[{
                'video_device': '/dev/video_wrist_right',
                'pixel_format': 'mjpeg2rgb',
                'image_width': 640,
                'image_height': 360,
                'framerate': 30.0,
            }],
            output={'both': 'log'},
        )
    )
    
    # Real Arms
    ld.add_action(
        Node(
            package=package_name,
            executable="lift_node",
            namespace='left/lift',
            parameters=[{
                'device': '/dev/tty_puppet_lift_left',
                'side': 'left',
            }],
            remappings=[
                ('joint_states', '/joint_states'),
            ],
        )
    )
    
    ld.add_action(
        Node(
            package=package_name,
            executable="arm_node",
            namespace='left/arm',
            parameters=[{
                'device': '/dev/tty_puppet_left',
                'side': 'left',
            }],
            remappings=[
                ('joint_states', '/joint_states'),
            ],
        )
    )
    
    ld.add_action(
        Node(
            package=package_name,
            executable="lift_node",
            namespace='right/lift',
            parameters=[{
                'device': '/dev/tty_puppet_lift_right',
                'side': 'right',
            }],
            remappings=[
                ('joint_states', '/joint_states'),
            ],
        )
    )
    
    ld.add_action(
        Node(
            package=package_name,
            executable="arm_node",
            namespace='right/arm',
            parameters=[{
                'device': '/dev/tty_puppet_right',
                'side': 'right',
            }],
            remappings=[
                ('joint_states', '/joint_states'),
            ],
            output='screen',
            emulate_tty=True,
        )
    )
    
    # # Fake Arms
    # ld.add_action(
    #     Node(
    #         package=package_name,
    #         executable="dry_run_node",
    #         namespace='left',
    #         parameters=[{
    #             'side': 'left',
    #         }],
    #         remappings=[
    #             ('joint_states', '/joint_states'),
    #         ],
    #     )
    # )
    
    # ld.add_action(
    #     Node(
    #         package=package_name,
    #         executable="dry_run_node",
    #         namespace='right',
    #         parameters=[{
    #             'side': 'right',
    #         }],
    #         remappings=[
    #             ('joint_states', '/joint_states'),
    #         ],
    #     )
    # )
    
    # IK
    ld.add_action(
        Node(
            package=package_name,
            executable="ik_node",
            namespace='left',
            parameters=[{
                'side': 'left',
            }],
        )
    )
    
    ld.add_action(
        Node(
            package=package_name,
            executable="ik_node",
            namespace='right',
            parameters=[{
                'side': 'right',
            }],
        )
    )
    
    # Base
    ld.add_action(
        Node(
            package=package_name,
            executable="base_node",
            parameters=[{
                'device': 'can0',
            }],
        )
    )
    
    # Teleop (Web)
    ld.add_action(
        Node(
            package=package_name,
            executable="teleop_web_node",
        )
    )
    
    # Visualization
    ld.add_action(
        IncludeLaunchDescription(
            FrontendLaunchDescriptionSource([
                str(Path(get_package_share_directory('astra_description'))
                    / 'launch' / 'publish_model.launch'), 
            ]),
        )
    )

    # ld.add_action(
    #     Node(
    #         package="rviz2",
    #         executable="rviz2",
    #         respawn=False,
    #         arguments=["-d", LaunchConfiguration("rviz_config")],
    #         output={'both': 'log'},
    #     )
    # )

    return ld
