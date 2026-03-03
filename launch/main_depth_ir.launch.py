import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 1. 相机节点
        Node(
            package='realsense2_camera',
            executable='realsense2_camera_node',
            namespace='camera',
            name='camera',
            parameters=[{
                'initial_reset': True,
                'enable_color': False,
                'enable_depth': True,
                'enable_infra1': True,
                'enable_infra2': False,
                'depth_module.profile': '848x480x30', 
                'infra_width': 848,
                'infra_height': 480,
                'infra_fps': 30,
                'align_depth.enable': False,
                'emitter_enabled': False,           # 🔴 关键：关闭散斑
                'depth_module.enable_auto_exposure': True, # 白天用自动曝光即可
                'enable_gyro': False,
                'enable_accel': False,
            }],
            output='screen'
        ),

        # 2. 视觉追踪节点 (输出目标坐标)
        Node(
            package='human_tracker',
            executable='depth_tracker',
            name='depth_tracker_node',
            output='screen'
        ),

        # 3. 🐶 机器狗跟随节点 (接收坐标，输出 cmd_vel)
        Node(
            package='human_tracker',
            executable='go2_follower',
            name='go2_follower_node',
            output='screen'
        ),

        # 4. 可视化节点 (按需开启，如果在真实机器狗上跑，为了省算力可以注释掉)
        Node(
            package='rqt_image_view',
            executable='rqt_image_view',
            arguments=['/human_tracker/output'],
            output='screen'
        )
    ])