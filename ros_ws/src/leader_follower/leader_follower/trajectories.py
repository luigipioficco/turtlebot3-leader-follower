# The route the target follows.
#
# ROS-free module: this is data, not logic, so it can be inspected and checked
# without starting rclpy.

TRAJECTORIES = {

    # Fourteen turns, weaving through the chicane. The turns are what
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
