from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Node(
        #     package='sno',
        #     executable='fake_zone_node',
        #     name='fake_zone_node',
        #     output='screen'),
        # Node(
        #     package='sno',
        #     executable='fake_location_heading_node',
        #     name='fake_location_heading_node',
        #     output='screen'),
        # Node(
        #     package='sno',
        #     executable='fake_control_node',
        #     name='fake_control_node',
        #     output='screen'),
        # Node(
        #     package='sno',
        #     executable='compass_node',
        #     name='compass_node',
        #     output='screen'),
        Node(
            package='sno',
            executable='control_node',
            name='control_node',
            output='screen'),
        Node(
            package='sno',
            executable='flutter_node',
            name='flutter_node',
            output='screen'),
        # Node(
        #     package='sno',
        #     executable='gps_node',
        #     name='gps_node',
        #     output='screen'),
        Node(
            package='sno',
            executable='handle_node',
            name='handle_node',
            output='screen'),
        Node(
            package='sno',
            executable='mode_node',
            name='mode_node',
            output='screen'),
        Node(
            package='sno',
            executable='motor_node',
            name='motor_node',
            output='screen'),
        # Node(
        #     package='sno',
        #     executable='navigation_node',
        #     name='navigation_node',
        #     output='screen'),
        Node(
            package='sno',
            executable='stepper_node',
            name='stepper_node',
            output='screen'),
        # Node(
        #     package='sno',
        #     executable='ultrasonic_node',
        #     name='ultrasonic_node',
        #     output='screen'),
        # Node(
        #     package='sno',
        #     executable='waypoint_node',
        #     name='waypoint_node',
        #     output='screen'),
        ])