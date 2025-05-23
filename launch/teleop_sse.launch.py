
from launch import LaunchDescription
from launch_ros.actions import Node

import os
from ament_index_python.packages import get_package_share_directory

pkg_share = get_package_share_directory('tello_pilot')
rviz_config = os.path.join(
    pkg_share,
    'config',
    'teleop_sse_config.rviz'
)


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='opencv_cam',
            executable='opencv_cam_main',
            name='opencv_cam',
            output='screen',
            parameters=[
                # {'index': 0},
                {'index': 4},           # /dev/video4
                {'image_width': 640},
                {'image_height': 480},
                {'framerate': 25},
                {'camera_frame_id': 'camera_frame'},
            ]
        ),
        Node(
            package='joy',
            executable='joy_node',
            output='screen',
        ),
        Node(
            package='tello_driver',
            executable='tello_driver_main',
            output='screen',
        ),
        Node(
            package='tello_pilot',
            executable='ar_detector',
            output='screen',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config]
        )
    ])





## memo
# <node pkg="usb_cam" executable="usb_cam_node_exe" name="usb_cam" output="screen">
        #     <param name="video_device" value="/dev/video0"/>
        #     <param name="image_width" value="640"/>
        #     <param name="image_height" value="480"/>
        #     <param name="pixel_format" value="mjpeg2rgb"/>
        #     <param name="framerate" value="25.0"/>
        # </node>
# <node pkg="tello_driver" executable="tello_joy_main" output="screen" />

