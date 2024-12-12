from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='example_package',
            executable='image_downscaler',
            name='image_downscaler'
        ),

        Node(
            package='example_package',
            executable='bram_node',
            name='bram_node'
        ),

        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='usb_cam_node_exe'
        ),

        Node(
            package='draw_with_motors',
            executable='draw_node',
            name='draw_node'
        )
    ])