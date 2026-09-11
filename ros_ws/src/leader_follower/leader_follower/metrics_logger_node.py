# Records to CSV everything needed to compare the two strategies.
#
# Two different distances, and the difference matters:
import csv
import math
import os
from datetime import datetime

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32, String


class MetricsLogger(Node):

    def __init__(self):
        super().__init__('metrics_logger_node')

        self.declare_parameter('output_dir', '/root/ros_workspace/results')
        self.declare_parameter('run_label', 'run')
        self.declare_parameter('desired_distance', 1.50)
        self.declare_parameter('collision_threshold', 0.16)
        # Run duration in SIMULATED time, 0 = unlimited.
        #
        # This node must end the run, not a timer in the launch file: that one
        # counts wall-clock seconds, and Gazebo runs at a varying fraction of
        # real time, so the two strategies would face different amounts of
        # scenario. See check_duration.
        self.declare_parameter('duration', 0.0)
        self.declare_parameter('min_valid_range', 0.12)
        self.declare_parameter('sample_rate', 20.0)
        self.declare_parameter('lost_timeout', 0.40)
        # [x, y, yaw] in the world frame; must match the values in the launch
        # file, which are the ones actually used to spawn the robots.
        self.declare_parameter('follower_spawn', [1.7, 2.0, 3.1416])
        self.declare_parameter('target_spawn', [0.5, 2.0, 3.1416])

        self.out_dir = self.get_parameter('output_dir').value
        self.label = self.get_parameter('run_label').value
        self.d_des = float(self.get_parameter('desired_distance').value)
        self.coll_thr = float(self.get_parameter('collision_threshold').value)
        self.duration = float(self.get_parameter('duration').value)
        self.min_range = float(self.get_parameter('min_valid_range').value)
        rate = float(self.get_parameter('sample_rate').value)
        self.dt = 1.0 / rate
        self.lost_timeout = float(self.get_parameter('lost_timeout').value)
        os.makedirs(self.out_dir, exist_ok=True)
        stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.prefix = os.path.join(self.out_dir, f"{self.label}_{stamp}")

        self.samples_path = self.prefix + '_samples.csv'
        self.fh = open(self.samples_path, 'w', newline='')
        self.csv = csv.writer(self.fh)
        # There is no 'true_distance' column: the distance recorded is the one
        # MEASURED by the camera, which is the only reliable source in this
        # system and is also the quantity the control loop acts on. A column
        # filled in only while the marker is visible is honest; inventing a
        # value when there is no measurement would flatter whichever strategy
        # loses the target more often.
        self.csv.writerow(['t', 'state', 'visible', 'meas_distance',
                           'distance_error', 'cmd_v', 'cmd_w',
                           'min_obstacle', 'path_length'])

        self.t0 = None
        self.state = 'SEARCH'
        self.last_pose_t = None
        self.last_meas = (float('nan'), float('nan'))
        self.cmd = (0.0, 0.0)
        self.min_obs = float('inf')
        self.path_len = 0.0

        self.rows = []
        self.since_flush = 0
        self.flush_every = 40          # every 2 s at 20 Hz
        self.loss_events = 0
        self.definitive = 0
        self.recoveries = []
        self.lost_at = None
        self.was_visible = False
        self.went_search = False

        self.create_subscription(PoseStamped, '/target/pose', self.on_pose, 10)
        self.create_subscription(String, '/follower/state', self.on_state, 10)
        # The measured distance comes from the controller, which computes and
        # uses it. Rebuilding it here from poses would require a position
        # source, and this simulator has no reliable one.
        self.create_subscription(Float32, '/follower/measured_distance',
                                 self.on_distance, 10)
        self.create_subscription(TwistStamped, '/cmd_vel', self.on_cmd, 10)
        self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)

        self.timer = self.create_timer(1.0 / rate, self.sample)
        self.get_logger().info(f"metrics_logger started | output={self.samples_path}")

    # ------------------------------------------------------------------ input

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_state(self, msg):
        self.state = msg.data

    def on_cmd(self, msg):
        self.cmd = (msg.twist.linear.x, msg.twist.angular.z)

    def on_scan(self, msg):
        r = np.array(msg.ranges, dtype=np.float64)
        # The message's own range_min arrives as zero from the gz bridge, so it
        # filters nothing: without an explicit floor, readings closer than the
        # sensor can physically see would be counted as obstacles.
        lo = max(float(msg.range_min), self.min_range)
        valid = np.isfinite(r) & (r > lo) & (r < msg.range_max)
        self.min_obs = float(np.min(r[valid])) if np.any(valid) else float('inf')

    def on_pose(self, msg):
        # Every message on /target/pose is a successful detection: the
        # detector publishes nothing when it cannot see the marker.
        t = self.now()
        self.last_pose_t = t
        if not self.was_visible and self.lost_at is not None:
            self.recoveries.append(t - self.lost_at)
            self.lost_at = None
            self.went_search = False
        self.was_visible = True

    def on_distance(self, msg):
        self.last_meas = (float(msg.data), 0.0)

    def visible_now(self):
        return (self.last_pose_t is not None
                and (self.now() - self.last_pose_t) <= self.lost_timeout)

    # ---------------------------------------------------------------- sampling

    def sample(self):
        # One sample per cycle: update the counters, write the row, check
        # whether the run is over. Three separate steps.
        t = self.now()
        if self.t0 is None:
            self.t0 = t
        rel = t - self.t0

        self.update_counters(t)
        self.write_row(rel)
        self.check_duration(rel)

    def update_counters(self, t):
        # Losses are counted from the TOPICS rather than by asking the
        # controller what state it is in: the instrument should not depend on
        # the system it measures more than necessary.
        if self.state == 'SEARCH' and self.lost_at is not None and not self.went_search:
            self.definitive += 1
            self.went_search = True

        # Path integrated from the command rather than from a position: it is
        # what the controller asked the platform to do, and it is bounded by
        # construction, so it cannot produce a jump.
        self.path_len += abs(self.cmd[0]) * self.dt

        if not self.visible_now() and self.was_visible:
            self.loss_events += 1
            self.lost_at = t
            self.went_search = False
            self.was_visible = False

    def write_row(self, rel):
        # The error is defined only while the target is visible: with no marker
        # there is no measurement, and inventing one would flatter the
        # strategy that loses it more often.
        vis = self.visible_now()
        md = self.last_meas[0] if vis else float('nan')
        derr = md - self.d_des if not math.isnan(md) else float('nan')

        self.csv.writerow([f"{rel:.3f}", self.state, int(vis),
                           f"{md:.4f}", f"{derr:.4f}",
                           f"{self.cmd[0]:.4f}", f"{self.cmd[1]:.4f}",
                           f"{self.min_obs:.4f}", f"{self.path_len:.4f}"])
        self.rows.append((rel, vis, derr, self.min_obs))

        # Periodic flush: Python buffers dozens of rows, and if a run is
        # interrupted without a clean SIGINT those rows are lost.
        self.since_flush += 1
        if self.since_flush >= self.flush_every:
            self.since_flush = 0
            self.fh.flush()

    def check_duration(self, rel):
        # The duration is decided HERE, on the simulation clock, and not by a
        # wall-clock timer in the launch file: Gazebo runs at a varying
        # fraction of real time, and a wall-clock timer would give the two
        # strategies different amounts of simulated time, which is the time
        # that actually matters for the comparison.
        if 0.0 < self.duration <= rel:
            self.get_logger().info(
                f"simulated duration reached ({rel:.1f} s), stopping the run")
            raise SystemExit          # spin() returns; main() writes the summary

    # ----------------------------------------------------------------- summary

    def write_summary(self):
        if not self.rows:
            return
        self.save_summary(self.compute_summary())

    @staticmethod
    def media(v):
        return round(float(np.mean(v)), 3) if len(v) else ''

    @staticmethod
    def rmse(v):
        return round(float(np.sqrt(np.mean(v ** 2))), 4) if v.size else ''

    def compute_summary(self):
        # rows are (t, visible, distance_error, min_obstacle)
        derr = np.array([r[2] for r in self.rows], dtype=float)
        obs = np.array([r[3] for r in self.rows], dtype=float)
        derr = derr[~np.isnan(derr)]
        obs = obs[np.isfinite(obs)]
        return {
            'run_label': self.label,
            'duration_s': round(self.rows[-1][0], 2),
            'tracking_loss_events': self.loss_events,
            'definitive_losses': self.definitive,
            'mean_recovery_time_s': self.media(self.recoveries),
            'rmse_distance_m': self.rmse(derr),
            'collisions': int(np.sum(obs < self.coll_thr)) if obs.size else 0,
        }

    def save_summary(self, summary):
        path = self.prefix + '_summary.csv'
        with open(path, 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(summary.keys())
            w.writerow(summary.values())
        self.get_logger().info(f"summary written to {path}")
        for k, v in summary.items():
            self.get_logger().info(f"  {k}: {v}")


def main(args=None):
    rclpy.init(args=args)
    node = MetricsLogger()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass          # SystemExit: simulated duration reached, not an error
    finally:
        node.write_summary()
        node.fh.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
