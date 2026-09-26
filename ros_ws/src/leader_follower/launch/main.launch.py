"""Complete system: simulation + perception + control + logging.

It includes the simulation launch file and adds the custom nodes on top.

Arguments:

    strategy:=predictive | reactive
    trajectory:=trajectory_1   (only route defined; kept explicit for clarity)
    gui:=true | false       Gazebo window
    rviz:=true | false
    duration:=<seconds>     stops the run on its own (0 = no limit)
    run_label:=<name>       configuration label written to the run_id/
                            run_label columns in results/summary.csv
                            and results/samples.csv

A comparable pair of runs:

    ros2 launch leader_follower main.launch.py
    ros2 launch leader_follower main.launch.py strategy:=reactive
"""
import os

from ament_index_python.packages import get_package_share_directory

from leader_follower import worlds
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            OpaqueFunction,
                            Shutdown, TimerAction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

def launch_setup(context, *args, **kwargs):
    """Spawn poses and the default trajectory are read from worlds.py and passed
    as numbers, and LaunchConfiguration values only resolve at runtime: hence
    OpaqueFunction."""
    pkg = get_package_share_directory('leader_follower')
    params_file = os.path.join(pkg, 'config', 'follower_params.yaml')
    rviz_config = os.path.join(pkg, 'rviz', 'leader_follower.rviz')


    strategy = LaunchConfiguration('strategy')

    # Converted here rather than with a substitution: PythonExpression yields
    # a STRING, and the logger declares 'duration' as a float. The wrong type
    # made the node die at start-up, and with on_exit=Shutdown it took the
    # whole run down with it after a second.
    duration_s = float(LaunchConfiguration('duration').perform(context))
    trajectory = (LaunchConfiguration('trajectory').perform(context)
                  or worlds.DEFAULT_TRAJECTORY)
    run_label = LaunchConfiguration('run_label')
    output_dir = LaunchConfiguration('output_dir')
    use_rviz = LaunchConfiguration('rviz')


    use_prediction = PythonExpression(["'", strategy, "' == 'predictive'"])

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg, 'launch', 'simulation.launch.py')),
        launch_arguments={'gui': LaunchConfiguration('gui')}.items())

    detector = Node(
        package='leader_follower', executable='target_detector_node',
        name='target_detector_node', output='screen',
        parameters=[params_file, {'use_sim_time': True}])

    # Working distance, overridable from the command line. At 1.5 m the
    # follower has 0.3 m of margin on the marker's detection range, which a
    # single turn is enough to consume: a strategy that stops falls out of
    # range, one that keeps advancing does not.
    ctrl_extra = {}
    fdist = LaunchConfiguration('follow_distance').perform(context)
    if fdist:
        ctrl_extra['desired_distance'] = float(fdist)

    controller = Node(
        package='leader_follower', executable='follower_controller_node',
        name='follower_controller_node', output='screen',
        parameters=[params_file, {'use_sim_time': True,
                                  'use_prediction': use_prediction,
                                  **ctrl_extra}])

    # The target is driven by the official Nav2 stack (AMCL + planner +
    # controller) in the /target namespace: same route, read from
    # trajectories.py. Speed and other limits are set once in
    # config/nav2_overrides.yaml, not overridable from the command line.
    driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg, 'launch', 'target_nav2.launch.py')),
        launch_arguments={'trajectory': trajectory}.items())

    logger = Node(
        package='leader_follower', executable='metrics_logger_node',
        name='metrics_logger_node', output='screen',
        parameters=[params_file, {'use_sim_time': True,
                                  'run_label': run_label,
                                  'output_dir': output_dir,
                                  'duration': duration_s}],
        # When the logger exits because the SIMULATED duration is reached,
        # the whole run shuts down. It decides, not a wall-clock timer.
        on_exit=Shutdown(reason='simulated run duration reached'))

    # With duration > 0 the run ends by itself. On shutdown the nodes receive
    # SIGINT and metrics_logger_node writes its summary in the finally block
    # before exiting.
    # Wall-clock safety net: if the simulation hangs, the logger never
    # advances in simulated time and the run would stay up forever. The factor
    # of 5 is deliberately generous, because under normal conditions it is the
    # logger that must end the run, not this.
    auto_stop = TimerAction(
        period=duration_s * 5.0,
        actions=[Shutdown(reason='run duration reached')],
        condition=IfCondition(
            PythonExpression([LaunchConfiguration('duration'), ' > 0'])))

    rviz = Node(
        package='rviz2', executable='rviz2', name='rviz2', output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(use_rviz))

    return [gazebo, rviz, detector, controller, driver, logger, auto_stop]

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'follow_distance', default_value='',
            description='following distance in m; empty = value from the '
                        'configuration file'),
        DeclareLaunchArgument('strategy', default_value='predictive',
                              description="'predictive' or 'reactive'"),
        DeclareLaunchArgument('trajectory', default_value='',
                              description='empty = default trajectory'),
        DeclareLaunchArgument('run_label', default_value='run'),
        DeclareLaunchArgument('output_dir', default_value='/root/ros_workspace/results'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('duration', default_value='0',
                              description='run duration in seconds, 0 = unlimited'),
        DeclareLaunchArgument('gui', default_value='true',
                              description='Gazebo window; false for batch runs'),
        OpaqueFunction(function=launch_setup),
    ])
