# The two routes the target follows.
#
# ROS-free module: this is data, not logic, so it can be inspected and checked
# without starting rclpy.

TRAJECTORIES = {

    # Twelve 90-degree turns in 29 m, weaving through the chicane: it passes
    # over the tip of one blade and under the tip of the other. The turns are
    # what make the marker go edge-on, so this is the route on which the
    # difference between the two strategies is actually observable.
    'trajectory_1': [
        (0.6, -3.0), (0.6, 0.0), (4.2, 0.0), (4.2, 2.0),
        (0.0, 2.0), (0.0, 3.0), (-0.6, 3.0), (-0.6, 0.0),
        (-4.2, 0.0), (-4.2, -2.0), (0.0, -2.0), (0.0, -3.0),
    ],

    # Four turns instead of twelve, in the free quadrant beside the chicane.
    # The marker stays almost always frontal, so losses are rare: this is the
    # route used to check that the following is stable.
    'trajectory_2': [
        (4.1, 3.4), (-1.0, 3.4), (-1.0, -0.2), (4.1, -0.2),
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
