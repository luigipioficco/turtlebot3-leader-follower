"""Generates the static occupancy map used by Nav2 to localise the target.

Only the TARGET uses this map (the follower stays map-free). Like the world
file, the map is generated from the geometry in worlds.py rather than drawn or
recorded with SLAM, so the arena Gazebo simulates and the map AMCL localises
against cannot diverge.

Output: src/leader_follower/maps/leader_follower_simple.{pgm,yaml}
(nav2_map_server format: 0 = occupied, 254 = free).
"""
import os
import sys

sys.path.insert(0, os.path.join(
    os.path.dirname(os.path.abspath(__file__)), '..', 'src', 'leader_follower'))

from leader_follower import worlds  # noqa: E402
from generate_world import WALL_T  # noqa: E402

RESOLUTION = 0.05          # m per cell, the Nav2 default
MARGIN = 0.5               # m of occupied border around the perimeter
MAP_NAME = 'leader_follower_simple'


def occupied(x, y):
    # Perimeter: the wall boxes are centred on RECT with thickness WALL_T,
    # so the inner face is at RECT - WALL_T/2. Everything beyond it is
    # unreachable and marked occupied.
    half_w, half_h = worlds.RECT
    if abs(x) >= half_w - WALL_T / 2 or abs(y) >= half_h - WALL_T / 2:
        return True
    # Chicane blades, with the same distance function used to validate the
    # trajectories.
    if any(worlds._dist_wall(wx, y0, y1, x, y) <= 0.0
           for wx, y0, y1 in worlds.WALLS):
        return True
    # The ArUco wall panels are deliberately NOT in the map: they are mounted
    # 0.3-0.5 m above the ground, above the LiDAR scan plane, so the scan never
    # sees them and AMCL must not expect them.
    return False


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    outdir = os.path.join(here, '..', 'src', 'leader_follower', 'maps')
    os.makedirs(outdir, exist_ok=True)

    half_w, half_h = worlds.RECT
    x0, y0 = -half_w - MARGIN, -half_h - MARGIN
    w = int(round(2 * (half_w + MARGIN) / RESOLUTION))
    h = int(round(2 * (half_h + MARGIN) / RESOLUTION))

    rows = []
    for r in range(h):
        # PGM row 0 is the TOP of the image, i.e. the largest y
        y = y0 + (h - 1 - r + 0.5) * RESOLUTION
        rows.append(bytes(
            0 if occupied(x0 + (c + 0.5) * RESOLUTION, y) else 254
            for c in range(w)))

    pgm = os.path.join(outdir, MAP_NAME + '.pgm')
    with open(pgm, 'wb') as f:
        f.write(f'P5\n{w} {h}\n255\n'.encode())
        f.write(b''.join(rows))

    yml = os.path.join(outdir, MAP_NAME + '.yaml')
    with open(yml, 'w', encoding='utf-8') as f:
        f.write(f'image: {MAP_NAME}.pgm\n'
                'mode: trinary\n'
                f'resolution: {RESOLUTION}\n'
                f'origin: [{x0}, {y0}, 0.0]\n'
                'negate: 0\n'
                'occupied_thresh: 0.65\n'
                'free_thresh: 0.25\n')
    print(f'written {os.path.normpath(pgm)} ({w}x{h}) and {os.path.basename(yml)}')


if __name__ == '__main__':
    main()
