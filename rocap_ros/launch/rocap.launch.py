from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction, DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    sim = LaunchConfiguration('sim', default='true')
    velodyne = LaunchConfiguration('velodyne', default='false')

    rocap_demo = [
        "xterm", "-hold", "-T", "simulationAndBridge", "-e",
        "ros2", "launch", "rocap_ros", "rocap_demo.launch.py"
    ]

    rocap_bridge = [
        "xterm", "-hold", "-T", "apiBridge", "-e",
        "ros2", "launch", "rocap_ros", "rocap_bridge.launch.py"
    ]

    slam_command = [
        "xterm", "-hold", "-T", "slam", "-e",
        "ros2", "launch", "rocap_ros", "slam_lidar_rgbd.launch.py",
        f"use_sim_time:={bool(sim)}",
        f"sim:={bool(sim)}",
    ]

    moving_base_command = [
        "xterm", "-hold", "-T", "moving base controller", "-e",
        "ros2", "run", "rocap_ros", "MovingBaseController.py"
    ]

    navigation_command = [
        "xterm", "-hold", "-T", "navigation", "-e",
        "ros2", "launch", "rocap_ros", "navigation.launch.py",
        f"use_sim_time:={bool(sim)}"
    ]

    pc_assembler_command = [
        "xterm", "-hold", "-T", "pc_assembler", "-e",
        "ros2", "launch", "rocap_ros", "pc_assembler.launch.py"
    ]

    velodyne_driver_command = [
        "xterm", "-hold", "-T", "velodyne_driver", "-e",
        "ros2", "launch", "rocap_ros", "velodyne_driver.launch.py"
    ]

    return LaunchDescription([
        DeclareLaunchArgument(
            'sim',
            default_value='true',
            description='Flag to indicate if simulation is running'
        ),
        DeclareLaunchArgument(
            'velodyne',
            default_value='false',
            description='Flag to indicate whether to use velodyne or simulated lidar'
        ),
        ExecuteProcess(
            cmd=rocap_demo,
            condition=IfCondition(sim)
        ),
        ExecuteProcess(
            cmd=moving_base_command
        ),
        ExecuteProcess(
            cmd=rocap_bridge,
            condition=UnlessCondition(sim)
        ),
        TimerAction(
            period=10.0,
            actions=[ExecuteProcess(cmd=slam_command)],
            condition=UnlessCondition(velodyne)
        ),
        TimerAction(
            period=10.0,
            actions=[ExecuteProcess(cmd=navigation_command)],
            # condition=UnlessCondition(sim)
        ),
        TimerAction(
            period=10.0,
            actions=[ExecuteProcess(cmd=pc_assembler_command)],
            condition=IfCondition(velodyne)
        ),
        TimerAction(
            period=15.0,
            actions=[ExecuteProcess(cmd=velodyne_driver_command)],
            condition=IfCondition(velodyne)
        ),
    ])
