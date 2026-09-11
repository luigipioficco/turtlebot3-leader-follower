# TurtleBot3 Leader-Follower Navigation with Visual Target Recovery

A vision-based following system in ROS2. A TurtleBot3 tracks a second one
carrying an ArUco marker, keeps a set distance while avoiding obstacles, and
**recovers the target after a loss of visibility** by estimating where it was
heading and continuing the pursuit blind for a few seconds.

No SLAM, no Nav2, no map: navigation is entirely reactive and perception-driven.

---

## Getting started

Everything runs inside a Docker container, so nothing needs to be installed on
the host besides Docker itself.

**1. Build the image** (once):

```bash
cd docker_ws && ./build.sh && cd ..
```

**2. Start the container:**

```bash
./run.sh
```

**3. Build and launch, inside the container:**

```bash
source /opt/ros/jazzy/setup.bash
export TURTLEBOT3_MODEL=burger
cd /root/ros_workspace
colcon build --symlink-install
source install/setup.bash
ros2 launch leader_follower main.launch.py
```

Gazebo and RViz2 open side by side. RViz shows the camera view with the
detected marker, the laser scan, and the current state of the follower.

> `./exec.sh` opens another shell into the running container. Each new shell
> needs `source /root/ros_workspace/install/setup.bash`.

> Worlds, models and configuration files are **copied** into `install/`, not
> symlinked. After changing one of those, run `colcon build` again.

---

## Comparing the two strategies

The point of the project is the comparison between a follower that predicts
where the target went and one that simply stops. Run these one after the other,
and give each a couple of minutes: a full lap of the route takes over two, and
the interesting behaviour is what happens at the turns.

```bash
# predictive: keeps advancing on the estimate and re-acquires the target
ros2 launch leader_follower main.launch.py

# reactive: stops, rotates on the spot, and does not find the target again
ros2 launch leader_follower main.launch.py strategy:=reactive
```

The follower works at 1.5 m against a detection range of about 1.8, and the
target moves at 0.21 m/s against a follower limited to 0.22. Those margins are
deliberately narrow: a follower that stops falls out of detection range and
does not recover. At a more comfortable 1.2 m the two strategies would be hard
to tell apart by eye.

> The follower never gives up permanently: after a full lap the target comes
> round again and is re-acquired even by the reactive strategy. What the two
> differ in is how much of the run is spent rotating on the spot in between.

---

## Launch arguments

| argument | values | default | meaning |
|---|---|---|---|
| `strategy` | `predictive`, `reactive` | `predictive` | with or without motion prediction |
| `trajectory` | `trajectory_1`, `trajectory_2` | `trajectory_1` | route driven by the target |
| `duration` | seconds | `0` | auto-shutdown, `0` = unlimited |
| `gui` | `true`, `false` | `true` | Gazebo window; `false` for batch runs |
| `rviz` | `true`, `false` | `true` | RViz2 |
| `target_speed` | m/s | from config | overrides the target speed |
| `follow_distance` | m | from config | overrides the following distance |

---

## Running the experiment

```bash
cd /root/ros_workspace
bash scripts/run_experiments.sh
```

Four runs of each strategy, 120 s of simulated time each, without graphics.
About half an hour. The script prints a comparison table at the end and writes
everything to `results/`.

Two details make the runs comparable. The strategies are **interleaved** rather
than run in blocks, because Gazebo leaves processes behind that affect the
following run and the effect accumulates. And each run is stopped on the
**simulation** clock rather than by a wall-clock timer, since Gazebo runs at a
variable fraction of real time.

Metrics: `tracking_loss_events`, `definitive_losses`, `mean_recovery_time_s`,
`rmse_distance_m`, `collisions`. The second is the one that carries the result:
it counts how many losses escalated to the blind search, which is what the
prediction is meant to reduce.

---

## Tests

The arena geometry lives in one place, `worlds.py`: the world file is generated
from it and the trajectories are checked against it. Those checks need no ROS
and no container:

```bash
cd ros_ws/src/leader_follower
python3 -m pytest test/ -q
```

They verify that every waypoint and every straight leg between waypoints keeps
clear of the walls, the chicane blades and the ArUco panels, that no leg is so
short that the target never leaves its deceleration taper, and that both robots
spawn in free space within detection range of each other. Run them after
touching `worlds.py` or `trajectories.py`: an invalid waypoint produces no error
at runtime, only a target that drives into a wall.

---

## What is in the repository

```
docker_ws/            Docker image and helper scripts
ros_ws/
├── src/
│   ├── turtlebot3_gazebo/     official ROBOTIS package, unmodified
│   └── leader_follower/       everything written for this project
│       ├── leader_follower/   the four nodes and the ROS-free modules
│       ├── launch/            main.launch.py and simulation.launch.py
│       ├── config/            every tunable parameter, with its rationale
│       ├── models/ worlds/    arena, robots and ArUco panels
│       └── test/              geometric validation of the trajectories
└── scripts/
    ├── run_experiments.sh     runs the experiment
    ├── analyze_results.py     comparison table and figures
    └── generate_world.py      regenerates the arena from worlds.py
run.sh                start the container
exec.sh               open another shell into it
chown_me.sh           fix file ownership after working as root inside
```

Inside `leader_follower`, the four nodes are:

| node | what it does |
|---|---|
| `target_detector_node` | finds the ArUco marker and publishes where the target is |
| `follower_controller_node` | control law, state machine, obstacle avoidance |
| `target_driver_node` | drives the target along a fixed route |
| `metrics_logger_node` | records every run to CSV |

Plus `geometry.py`, `trajectories.py`, `worlds.py` and `aruco_compat.py`, which
hold pure functions and data with no ROS dependency, and `config/follower_params.yaml`,
which collects every tunable parameter together with the reason for its value.

---

## References

- [TurtleBot3 documentation](https://emanual.robotis.com/docs/en/platform/turtlebot3/overview/)
- [OpenCV ArUco module](https://docs.opencv.org/4.x/d5/dae/tutorial_aruco_detection.html)
- [ros_gz bridge](https://github.com/gazebosim/ros_gz)
- [Gazebo Harmonic](https://gazebosim.org/docs/harmonic)
