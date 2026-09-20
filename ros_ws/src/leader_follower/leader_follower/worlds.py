# Definition of the test environment.
#
# ROS-free module: this is data, not logic, so test_waypoints.py can validate
# every trajectory against the geometry of the arena without starting rclpy.

import math

# inner half-extents of the rectangular perimeter
RECT = (5.0, 4.0)

# Chicane blades: (x, y_min, y_max), thickness 0.10. One hangs from the top
# edge and one rises from the bottom, so the free gap alternates and the route
# is forced to weave. No obstacles scattered at random: every element has
# a function.
WALLS = [(-1.6, 0.60, 4.00), (1.6, -4.00, -0.60)]

# ArUco panels: (x, y, yaw, id), flush against the walls. The panel is thin
# along Y, so with yaw 0 its textured face looks along +/-y; the wrong yaw
# leaves it edge-on against the wall like a blade, and the detector never sees
# it, with nothing to indicate that anything is wrong.
MARKERS = [(0.0, -3.94, 0.0, 0), (0.0, 3.94, 0.0, 1), (4.94, 0.0, 1.5708, 2)]

# Spawn poses (x, y, yaw). At the mouth of the chicane, not in the middle of
# the arena: the run starts on the route itself instead of with a transfer leg.
TARGET_SPAWN = (0.5, 2.0, 3.1416)
FOLLOWER_SPAWN = (1.7, 2.0, 3.1416)

WORLD_FILE = 'leader_follower_simple.world'
DEFAULT_TRAJECTORY = 'trajectory_1'


def _dist_markers(x, y):
    """The ArUco panels are obstacles like any other."""
    return min(math.hypot(x - mx, y - my) - 0.14 for mx, my, _, _ in MARKERS)


def _dist_wall(wx, y0, y1, x, y):
    """Distance from a vertical blade, half-thickness included."""
    dx = abs(x - wx) - 0.05
    dy = max(y0 - y, 0.0, y - y1)
    if dx > 0.0 or dy > 0.0:
        return math.hypot(max(dx, 0.0), dy)
    return -min(abs(dx), abs(dy))       # inside the blade


def clearance(x, y):
    # Distance to the nearest obstacle boundary, negative when inside one.
    #
    # This is the single source of truth for the arena geometry: the world
    # file is generated from it, and the same function is used to verify that
    # no trajectory waypoint falls inside an obstacle. Describing the geometry
    # in two places is how the world and the check come to disagree.
    return min(_dist_markers(x, y),
               min(_dist_wall(wx, y0, y1, x, y) for wx, y0, y1 in WALLS),
               RECT[0] - abs(x), RECT[1] - abs(y))
