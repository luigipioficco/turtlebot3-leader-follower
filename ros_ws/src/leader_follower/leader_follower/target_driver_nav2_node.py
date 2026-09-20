#! /usr/bin/env python3
# Copyright 2021 Samsung Research America
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Modified 2026 by Luigi Pio Ficco for the leader_follower project.
# Original: ros-navigation/navigation2, branch jazzy,
#   nav2_simple_commander/nav2_simple_commander/demo_security.py
# Changes (everything else is unchanged):
#   - the route is read from leader_follower/trajectories.py and started from
#     the first waypoint ahead of the spawn pose in worlds.py
#   - node in the /target namespace, initial pose = worlds.TARGET_SPAWN
#   - each pose points along the next leg, instead of all facing +x
#   - the route is a closed loop, so it is repeated instead of reversed
#   - timeout raised from 180 s to 400 s (one lap of trajectory_1 is ~142 s)

from copy import deepcopy
import math

from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

import rclpy
from rclpy.duration import Duration

from leader_follower import trajectories
from leader_follower.worlds import DEFAULT_TRAJECTORY, TARGET_SPAWN


"""
Basic security route patrol demo. In this demonstration, the expectation
is that there are security cameras mounted on the robots recording or being
watched live by security staff.
"""


def main():
    rclpy.init()

    navigator = BasicNavigator(node_name='target_driver_nav2_node', namespace='target')

    # Route from trajectories.py, started from the nearest waypoint ahead of
    # the spawn pose, so the target does not turn back towards the follower.
    name = navigator.declare_parameter('trajectory', DEFAULT_TRAJECTORY).value
    route = list(trajectories.get(name))
    x0, y0, yaw0 = TARGET_SPAWN
    ahead = [i for i, (x, y) in enumerate(route)
             if math.cos(math.atan2(y - y0, x - x0) - yaw0) > 0.0]
    start = min(ahead or range(len(route)), key=lambda i: math.hypot(route[i][0] - x0, route[i][1] - y0))
    security_route = route[start:] + route[:start]

    # Set our demo's initial pose
    initial_pose = PoseStamped()
    initial_pose.header.frame_id = 'map'
    initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    initial_pose.pose.position.x = x0
    initial_pose.pose.position.y = y0
    initial_pose.pose.orientation.z = math.sin(yaw0 / 2.0)
    initial_pose.pose.orientation.w = math.cos(yaw0 / 2.0)
    navigator.setInitialPose(initial_pose)

    # Wait for navigation to fully activate
    navigator.waitUntilNav2Active()

    # Do security route until dead
    while rclpy.ok():
        # Send our route
        route_poses = []
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = navigator.get_clock().now().to_msg()
        for k, pt in enumerate(security_route):
            pose.pose.position.x = pt[0]
            pose.pose.position.y = pt[1]
            nx, ny = security_route[(k + 1) % len(security_route)]
            yaw = math.atan2(ny - pt[1], nx - pt[0])
            pose.pose.orientation.z = math.sin(yaw / 2.0)
            pose.pose.orientation.w = math.cos(yaw / 2.0)
            route_poses.append(deepcopy(pose))
        navigator.goThroughPoses(route_poses)

        # Do something during our route (e.x. AI detection on camera images for anomalies)
        # Simply print ETA for the demonstation
        i = 0
        while not navigator.isTaskComplete():
            i += 1
            feedback = navigator.getFeedback()
            if feedback and i % 5 == 0:
                print(
                    'Estimated time to complete current route: '
                    + '{0:.0f}'.format(
                        Duration.from_msg(feedback.estimated_time_remaining).nanoseconds
                        / 1e9
                    )
                    + ' seconds.'
                )

                # Some failure mode, must stop since the robot is clearly stuck
                if Duration.from_msg(feedback.navigation_time) > Duration(
                    seconds=400.0
                ):
                    print('Navigation has exceeded timeout of 400s, canceling request.')
                    navigator.cancelTask()

        # Closed loop: the last waypoint connects back to the first, so the
        # same route is simply sent again
        result = navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            print('Route complete! Restarting...')
        elif result == TaskResult.CANCELED:
            print('Security route was canceled, exiting.')
            exit(1)
        elif result == TaskResult.FAILED:
            print('Security route failed! Restarting from other side...')

    exit(0)


if __name__ == '__main__':
    main()
