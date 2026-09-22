# The route the target follows.
#
# ROS-free module: this is data, not logic, so it can be inspected and checked
# without starting rclpy.

TRAJECTORIES = {

    # Fourteen turns in about 30 m, weaving through the chicane. Four of them
    # pass more than 1.5 m from any obstacle -- comfortably outside Nav2's
    # 0.35 m costmap inflation (config/nav2_overrides.yaml), so the target
    # goes through them at its normal, undamped speed. Two of those four,
    # (0.0, 2.0) and (0.0, -2.0), were already this open on the original
    # twelve-waypoint route; the other two, (2.0, 1.0) and (-2.0, -1.0), were
    # added specifically to double that count. The remaining ten turns sit
    # close enough to a wall or a chicane blade that Nav2 slows the target
    # down for them regardless of what the follower does. The turns are what
    # make the marker go edge-on, so this is the route on which the
    # difference between the two strategies is actually observable.
    'trajectory_1': [
        (0.6, -3.0), (0.6, 0.0), (2.0, 1.0), (4.2, 0.0), (4.2, 2.0),
        (0.0, 2.0), (0.0, 3.0), (-0.6, 3.0), (-0.6, 0.0), (-2.0, -1.0),
        (-4.2, 0.0), (-4.2, -2.0), (0.0, -2.0), (0.0, -3.0),
    ],
}


def get(name):
    # Waypoints of a route, with a clear error if it does not exist.
    #
    # Falling back on a default would be worse than failing: the target would
    # drive waypoints never validated against the arena geometry, and could
    # end up inside a wall with nothing in the log to say why.
    if name not in TRAJECTORIES:
        raise KeyError(
            f"unknown trajectory '{name}', available: {sorted(TRAJECTORIES)}")
    return TRAJECTORIES[name]
