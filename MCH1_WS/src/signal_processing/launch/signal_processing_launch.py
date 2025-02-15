# Import libraries 
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='signal_processing',
            executable='signal_generator',
            name='signal_generator',
            output='screen'
        ),
        Node(
            package='signal_processing',
            executable='process',
            name='process',
            output='screen'
        ),
        Node( # Launch plotjuggler to visualize the data 
            package='plotjuggler',
            executable='plotjuggler',
            name='plotjuggler',
            arguments=['--ros-args', '-r', '__node:=plotjuggler'],
            output='screen'
        ),
    ])
