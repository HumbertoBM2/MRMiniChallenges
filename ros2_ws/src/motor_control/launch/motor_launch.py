# Import libraries
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    package_name = 'motor_control'
    package_share_directory = get_package_share_directory(package_name)
    config_file = os.path.join(package_share_directory, 'config', 'controller_params.yaml')
    motor_node = Node( # Initialize dc motor node 
        name="motor_sys",
        package=package_name,
        executable='dc_motor',
        output='screen'
    )
    sp_node = Node( # Initialize set point node 
        name="sp_gen",
        package=package_name,
        executable='set_point',
        output='screen'
    )
    ctrl_node = Node( # Initialize controller node
        name="ctrl",
        package=package_name,
        executable='controller',
        output='screen',
        parameters=[config_file]
    )
    rqt_graph_node = ExecuteProcess( # Initialize rqt_graph
        cmd=["ros2", "run", "rqt_graph", "rqt_graph"],
        output="screen"
    )
    rqt_reconfigure_node = ExecuteProcess( # Initialize rqt_reconfigure
        cmd=["ros2", "run", "rqt_reconfigure", "rqt_reconfigure"],
        output="screen"
    )
    plotjuggler_node = ExecuteProcess( # Initialize PlotJuggler
        cmd=["ros2", "run", "plotjuggler", "plotjuggler"],
        output="screen"
    )
    return LaunchDescription([
        motor_node,
        sp_node,
        ctrl_node,
        rqt_graph_node,
        rqt_reconfigure_node,
        plotjuggler_node
    ])
