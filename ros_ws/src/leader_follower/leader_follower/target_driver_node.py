# Drives the target robot along predefined trajectories.
#
# Not an accessory: without repeatable trajectories the comparison between the
# reactive and the predictive strategy would be meaningless, because the two
# would face different scenarios. This is Challenge 5 of the proposal.
import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool

from leader_follower import trajectories
from leader_follower.worlds import DEFAULT_TRAJECTORY


class TargetDriver(Node):

    def __init__(self):
        super().__init__('target_driver_node')

        self.declare_parameter('trajectory', '')
        self.declare_parameter('max_linear', 0.21)
        self.declare_parameter('max_angular', 1.0)
        self.declare_parameter('k_angular', 1.6)
        self.declare_parameter('waypoint_tolerance', 0.18)
        self.declare_parameter('loop', True)
        self.declare_parameter('start_delay', 5.0)
        self.declare_parameter('pose_topic', '/target/ground_truth')
        self.declare_parameter('cmd_vel_topic', '/target/cmd_vel')
        # Safety net. The target has no sensors: if anything blocks it — a
        # bump from the follower, a corner, a slipping wheel — it keeps aiming
        # at the waypoint and pushes against the obstacle indefinitely. These
        # two parameters detect the lack of progress and skip the waypoint.
        self.declare_parameter('stuck_time', 6.0)       # [s] still while commanding forward
        self.declare_parameter('stuck_distance', 0.05)
        # Beyond this misalignment the waypoint is effectively behind, and
        # going for it would turn the target into the follower pursuing it.
        self.declare_parameter('behind_threshold', 2.36)   # [rad] 135 degrees

        name = self.get_parameter('trajectory').value or DEFAULT_TRAJECTORY
        try:
            self.waypoints = list(trajectories.get(name))
        except KeyError as e:
            # better to stop with a clear message than to start on a fallback:
            # the target would follow waypoints never validated against the
            # arena and could end up inside a wall, with no error message.
            self.get_logger().error(str(e))
            raise

        self.v_max = float(self.get_parameter('max_linear').value)
        self.w_max = float(self.get_parameter('max_angular').value)
        self.k_ang = float(self.get_parameter('k_angular').value)
        self.tol = float(self.get_parameter('waypoint_tolerance').value)
        self.loop = bool(self.get_parameter('loop').value)
        self.delay = float(self.get_parameter('start_delay').value)

        self.stuck_time = float(self.get_parameter('stuck_time').value)
        self.stuck_dist = float(self.get_parameter('stuck_distance').value)
        self.behind_threshold = float(self.get_parameter('behind_threshold').value)

        self.idx = None            # chosen on the first pose received
        self.pose = None
        self.t0 = None
        self.done = False
        self.ref_pose = None       # position at the start of the stuck window
        self.ref_t = None

        self.create_subscription(PoseStamped, self.get_parameter('pose_topic').value,
                                 self.on_pose, 10)
        self.pub = self.create_publisher(
            TwistStamped, self.get_parameter('cmd_vel_topic').value, 10)
        self.pub_done = self.create_publisher(Bool, '/target/trajectory_done', 10)

        self.create_timer(0.05, self.loop_cb)
        self.get_logger().info(
            f"target_driver started | route '{name}' "
            f"({len(self.waypoints)} waypoints, world coordinates) "
            f"| starting in {self.delay:.0f} s")

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_pose(self, msg):
        # Real pose published by the simulator, not odometry: odometry is
        # integrated from the wheel encoders and drifts at every turn, and
        # after a few laps it drove into a wall believing it had free space.
        q = msg.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.pose = (msg.pose.position.x, msg.pose.position.y, yaw)
        if self.t0 is None:
            # the start delay counts from the first pose received, not from
            # node start-up: before that there is nothing to drive towards,
            # and the delay would be spent waiting for the simulator instead.
            self.t0 = self.now()

    def start_score(self, i, x, y, yaw):
        """A starting candidate is valid if it lies ahead AND its outgoing leg
        does not require a reversal. Returns the distance, or None."""
        gx, gy = self.waypoints[i]
        to_wp = math.atan2(gy - y, gx - x)
        ahead = abs(math.atan2(math.sin(to_wp - yaw), math.cos(to_wp - yaw)))
        if ahead > math.pi / 2:
            return None

        # The turn is measured from the APPROACH direction, not from the
        # initial heading: by the time the robot arrives it points the way it
        # travelled.
        nx, ny = self.waypoints[(i + 1) % len(self.waypoints)]
        leg = math.atan2(ny - gy, nx - gx)
        turn = abs(math.atan2(math.sin(leg - to_wp), math.cos(leg - to_wp)))
        if turn > self.behind_threshold:
            return None
        return math.hypot(gx - x, gy - y)

    def pick_start(self, x, y, yaw):
        # Which waypoint to start from, based on where the robot is and
        # where the route goes from there.
        scores = [(d, i) for i in range(len(self.waypoints))
                  if (d := self.start_score(i, x, y, yaw)) is not None]
        if scores:
            d, i = min(scores)
        else:
            # none satisfies both: fall back on the nearest one and accept
            # the turn, which beats starting on a leg that runs away from the
            # spawn position.
            d, i = min((math.hypot(gx - x, gy - y), k)
                       for k, (gx, gy) in enumerate(self.waypoints))
        self.get_logger().info(
            f"starting from waypoint {i + 1}/{len(self.waypoints)} "
            f"{self.waypoints[i]}, {d:.2f} m away")
        return i

    def loop_cb(self):
        now = self.now()
        if self.pose is None or (now - self.t0) < self.delay or self.done:
            # A zero command is published EXPLICITLY here: a silent topic
            # leaves the last command active in the Gazebo plugin.
            self.publish(0.0, 0.0)
            return

        x, y, yaw = self.pose
        if self.idx is None:
            self.idx = self.pick_start(x, y, yaw)

        if self.advance_if_needed(x, y, yaw, now):
            return                      # trajectory complete

        self.publish(*self.command_towards(x, y, yaw))

    def advance_if_needed(self, x, y, yaw, now):
        # Three reasons to move to the next waypoint, each born of an observed
        # failure. Returns True when the trajectory is complete.
        gx, gy = self.waypoints[self.idx]

        # 1. reached: the normal case
        if math.hypot(gx - x, gy - y) < self.tol:
            self.get_logger().info(
                f"waypoint {self.idx + 1}/{len(self.waypoints)} reached")
            self.next_waypoint()
            if self.idx == 0 and not self.loop:
                self.done = True
                self.pub_done.publish(Bool(data=True))
                self.get_logger().info('trajectory completed')
                self.publish(0.0, 0.0)
                return True

        # 2. stuck: the target has no sensors, so if anything blocks it it
        #    would push against the obstacle indefinitely
        if self.is_stuck(x, y, now):
            self.get_logger().warn(
                f"stuck before waypoint {self.idx + 1}, skipping it")
            self.next_waypoint()

        # 3. behind: going for it would turn the target 180 degrees into the
        #    follower pursuing it a metre and a half back
        if abs(self.bearing_to_goal(x, y, yaw)) > self.behind_threshold:
            self.get_logger().warn(
                f"waypoint {self.idx + 1} is behind, skipping it")
            self.next_waypoint()
        return False

    def bearing_to_goal(self, x, y, yaw):
        gx, gy = self.waypoints[self.idx]
        heading = math.atan2(gy - y, gx - x)
        return math.atan2(math.sin(heading - yaw), math.cos(heading - yaw))

    def command_towards(self, x, y, yaw):
        # Same structure as the follower: proportional on the bearing error,
        # cosine on the misalignment, plus a factor that tapers the speed near
        # the waypoint.
        gx, gy = self.waypoints[self.idx]
        dist = math.hypot(gx - x, gy - y)
        err = self.bearing_to_goal(x, y, yaw)
        w = max(-self.w_max, min(self.w_max, self.k_ang * err))
        v = (self.v_max
             * max(0.0, math.cos(min(abs(err), math.pi / 2)))
             * min(1.0, dist / max(2 * self.tol, 1e-6)))
        return v, w

    def next_waypoint(self):
        self.idx = (self.idx + 1) % len(self.waypoints)
        self.ref_pose, self.ref_t = None, None

    def is_stuck(self, x, y, now):
        # True when the robot has not moved appreciably over the last
        # stuck_time seconds.
        if self.ref_pose is None:
            self.ref_pose, self.ref_t = (x, y), now
            return False
        if now - self.ref_t < self.stuck_time:
            return False
        moved = math.hypot(x - self.ref_pose[0], y - self.ref_pose[1])
        self.ref_pose, self.ref_t = (x, y), now
        return moved < self.stuck_dist

    def publish(self, v, w):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'target/base_footprint'
        msg.twist.linear.x = float(v)
        msg.twist.angular.z = float(w)
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = TargetDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.publish(0.0, 0.0)
        except Exception:
            pass          # context already gone: the stop is best effort
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
