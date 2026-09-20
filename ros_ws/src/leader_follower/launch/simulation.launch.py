"""Starts Gazebo, the world and the two robots.

Spawning and bridging are explicit rather than going through
spawn_turtlebot3.launch.py, because there are two robots here and that launch
file only knows about one.

On its own this moves nothing: it exists to check that the simulation is sound
before adding the logic on top.

The follower uses the standard topic names (cmd_vel, odom, scan, camera/...)
and its TF goes to /tf. The target lives entirely under /target and its TF is
NOT published on /tf: the follower does not need it, and it would create two
trees claiming the same frame names.
"""
import os

from ament_index_python.packages import get_package_share_directory

from leader_follower import worlds
from launch import LaunchDescription
from launch.actions import (AppendEnvironmentVariable, DeclareLaunchArgument,
                            IncludeLaunchDescription, OpaqueFunction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    """Builds the actions once the spawn poses are known.

    OpaqueFunction is needed because the poses are read from worlds.py and
    passed as numbers, and LaunchConfiguration values only resolve at runtime.
    """
    pkg = get_package_share_directory('leader_follower')
    tb3 = get_package_share_directory('turtlebot3_gazebo')
    ros_gz_sim = get_package_share_directory('ros_gz_sim')

    world = os.path.join(pkg, 'worlds', worlds.WORLD_FILE)
    urdf = os.path.join(pkg, 'urdf', 'follower.urdf')
    with open(urdf, 'r') as f:
        robot_desc = f.read()

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # Start poses, taken from worlds.py. The follower spawns behind the target
    # and already aligned with it, so the run begins in FOLLOW instead of
    # wasting time on an initial search.
    #
    # The target spawns off the route, at the point closest to the first
    # waypoint it will actually drive (target_driver_nav2_node.py picks the
    # entry point), so Nav2's own planner only has a short approach leg to
    # cover before the scripted route begins.
    fs, ts = worlds.FOLLOWER_SPAWN, worlds.TARGET_SPAWN

    f_x = LaunchConfiguration('follower_x').perform(context) or str(fs[0])
    f_y = LaunchConfiguration('follower_y').perform(context) or str(fs[1])
    f_yaw = LaunchConfiguration('follower_yaw').perform(context) or str(fs[2])
    t_x = LaunchConfiguration('target_x').perform(context) or str(ts[0])
    t_y = LaunchConfiguration('target_y').perform(context) or str(ts[1])
    t_yaw = LaunchConfiguration('target_yaw').perform(context) or str(ts[2])

    # Two paths: this project's models, and the official ROBOTIS ones that
    # provide the turtlebot3_common meshes both robots reference.
    set_resource_path = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH', os.path.join(pkg, 'models'))
    set_resource_path_tb3 = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH', os.path.join(tb3, 'models'))

    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': ['-r -s -v2 ', world],
                          'on_exit_shutdown': 'true'}.items())

    # With gui:=false Gazebo runs without its graphical client. That is by far
    # the largest saving, since the client redraws the whole scene every frame,
    # and in batch runs there is nobody watching.
    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': '-g -v2 ', 'on_exit_shutdown': 'true'}.items(),
        condition=IfCondition(LaunchConfiguration('gui')))

    spawn_follower = Node(
        package='ros_gz_sim', executable='create', output='screen',
        arguments=['-name', 'follower',
                   '-file', os.path.join(pkg, 'models', 'turtlebot3_follower', 'model.sdf'),
                   '-x', f_x, '-y', f_y, '-z', '0.01', '-Y', f_yaw])

    spawn_target = Node(
        package='ros_gz_sim', executable='create', output='screen',
        arguments=['-name', 'target',
                   '-file', os.path.join(pkg, 'models', 'turtlebot3_target', 'model.sdf'),
                   '-x', t_x, '-y', t_y, '-z', '0.01', '-Y', t_yaw])

    bridge_follower = Node(
        package='ros_gz_bridge', executable='parameter_bridge', output='screen',
        name='bridge_follower',
        arguments=['--ros-args', '-p',
                   f"config_file:={os.path.join(pkg, 'params', 'follower_bridge.yaml')}"])

    bridge_target = Node(
        package='ros_gz_bridge', executable='parameter_bridge', output='screen',
        name='bridge_target',
        arguments=['--ros-args', '-p',
                   f"config_file:={os.path.join(pkg, 'params', 'target_bridge.yaml')}"])

    # images go through a dedicated bridge (ros_gz_image), which handles the
    # image transport that the generic parameter_bridge does not
    bridge_image = Node(
        package='ros_gz_image', executable='image_bridge', output='screen',
        arguments=['/camera/image_raw'])

    rsp = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        name='robot_state_publisher', output='screen',
        parameters=[{'use_sim_time': use_sim_time, 'robot_description': robot_desc}])

    return [set_resource_path, set_resource_path_tb3, gzserver, gzclient,
            spawn_follower, spawn_target, bridge_follower, bridge_target,
            bridge_image, rsp]


def generate_launch_description():
    ld = LaunchDescription()
    for a, d in [('use_sim_time', 'true'), ('gui', 'true')]:
        ld.add_action(DeclareLaunchArgument(a, default_value=d))
    # Spawn poses default to worlds.py but stay overridable from the command
    # line; an empty default means "use the value from worlds.py".
    for a in ('follower_x', 'follower_y', 'follower_yaw',
              'target_x', 'target_y', 'target_yaw'):
        ld.add_action(DeclareLaunchArgument(a, default_value=''))
    ld.add_action(OpaqueFunction(function=launch_setup))
    return ld
