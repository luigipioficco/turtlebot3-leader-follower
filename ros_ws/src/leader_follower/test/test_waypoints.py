# Geometric validation of the trajectories against the arena.
#
# This is the second consumer of worlds.py, and the reason the geometry is
# written there once instead of being drawn directly in Gazebo: the world file
# is generated from it, and these checks are run against it. Describing the
# arena in two places is how the world and the check come to disagree, which
# already happened once: the world was modified, the check was not, and a robot
# spawned inside an obstacle with no error message at all.
#
# No ROS is involved, so this runs with a plain `pytest` and no sourced
# workspace.

import math

import pytest

from leader_follower import trajectories, worlds

# The TurtleBot3 Burger is 0.178 m wide, so 0.09 m is its half-width. A
# waypoint closer than this to an obstacle boundary cannot be occupied at all;
# the margin on top is what keeps the driver from grazing the blade while it
# tapers into the waypoint.
ROBOT_RADIUS = 0.09
MARGIN = 0.15
MIN_CLEARANCE = ROBOT_RADIUS + MARGIN

# The driver tapers its speed within twice the waypoint tolerance, so a leg
# shorter than that never reaches full speed and the target crawls.
MIN_LEG = 2 * 0.18


def all_routes():
    return sorted(trajectories.TRAJECTORIES)


@pytest.mark.parametrize('name', all_routes())
def test_waypoints_are_inside_the_arena(name):
    for i, (x, y) in enumerate(trajectories.get(name)):
        assert abs(x) < worlds.RECT[0] and abs(y) < worlds.RECT[1], \
            f"{name}: waypoint {i + 1} ({x}, {y}) is outside the perimeter"


@pytest.mark.parametrize('name', all_routes())
def test_waypoints_clear_every_obstacle(name):
    for i, (x, y) in enumerate(trajectories.get(name)):
        c = worlds.clearance(x, y)
        assert c > MIN_CLEARANCE, \
            f"{name}: waypoint {i + 1} ({x}, {y}) has {c:.3f} m of clearance, " \
            f"needs more than {MIN_CLEARANCE:.3f}"


@pytest.mark.parametrize('name', all_routes())
def test_legs_stay_clear_of_every_obstacle(name):
    # A waypoint can be clear while the straight leg joining it to the next one
    # passes through a blade. The legs are sampled every 5 cm.
    wps = trajectories.get(name)
    for i, (x0, y0) in enumerate(wps):
        x1, y1 = wps[(i + 1) % len(wps)]
        length = math.hypot(x1 - x0, y1 - y0)
        steps = max(int(length / 0.05), 1)
        for k in range(steps + 1):
            t = k / steps
            x, y = x0 + (x1 - x0) * t, y0 + (y1 - y0) * t
            c = worlds.clearance(x, y)
            assert c > ROBOT_RADIUS, \
                f"{name}: the leg {i + 1} -> {i + 2} passes at {c:.3f} m " \
                f"from an obstacle at ({x:.2f}, {y:.2f})"


@pytest.mark.parametrize('name', all_routes())
def test_legs_are_long_enough(name):
    wps = trajectories.get(name)
    for i, (x0, y0) in enumerate(wps):
        x1, y1 = wps[(i + 1) % len(wps)]
        length = math.hypot(x1 - x0, y1 - y0)
        assert length > MIN_LEG, \
            f"{name}: the leg {i + 1} -> {i + 2} is {length:.2f} m, " \
            f"shorter than {MIN_LEG:.2f}: the target would never leave the taper"


@pytest.mark.parametrize('name', all_routes())
def test_spawn_poses_are_clear(name):
    # Both robots must spawn in free space, and far enough apart that the
    # follower starts within detection range but not on top of the target.
    for label, pose in (('follower', worlds.FOLLOWER_SPAWN),
                        ('target', worlds.TARGET_SPAWN)):
        c = worlds.clearance(pose[0], pose[1])
        assert c > MIN_CLEARANCE, \
            f"the {label} spawns {c:.3f} m from an obstacle"

    separation = math.hypot(worlds.FOLLOWER_SPAWN[0] - worlds.TARGET_SPAWN[0],
                            worlds.FOLLOWER_SPAWN[1] - worlds.TARGET_SPAWN[1])
    assert 0.3 < separation < 1.8, \
        f"the robots spawn {separation:.2f} m apart, outside the detection range"


def test_unknown_trajectory_raises():
    # The driver relies on this: a silent fallback would send the target along
    # waypoints that these tests never validated.
    with pytest.raises(KeyError):
        trajectories.get('does_not_exist')
