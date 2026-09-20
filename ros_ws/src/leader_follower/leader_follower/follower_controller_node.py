# Follower controller: control law, state machine and safety arbitration.
#
#   FOLLOW  --marker lost-->  PREDICT  --horizon expired-->  SEARCH
#      ^                                                       |
#      +---------------------- marker seen ---------------------+

import math
from collections import deque

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import TwistStamped, PointStamped, PoseStamped
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String, Float32
import tf2_ros
import tf2_geometry_msgs  # noqa: F401  registers the PoseStamped transform

from leader_follower.geometry import estimate_velocity, predict_position

SEARCH, FOLLOW, PREDICT = 'SEARCH', 'FOLLOW', 'PREDICT'


class FollowerController(Node):

    def __init__(self):
        super().__init__('follower_controller_node')

        decl = self.declare_parameters('', [
            ('control_rate', 20.0),
            ('desired_distance', 1.50),
            ('distance_tolerance', 0.10),
            ('bearing_tolerance', 0.05),
            ('k_linear', 0.60),
            ('k_angular', 1.50),
            ('max_linear', 0.22),
            ('max_angular', 1.80),
            ('reverse_factor', 0.5),
            ('lost_timeout', 0.40),
            ('use_prediction', True),
            ('prediction_horizon', 3.0),
            ('velocity_window', 8),
            ('min_velocity_samples', 5),
            ('max_target_speed', 0.25),
            ('min_speed_for_prediction', 0.03),
            ('safety_distance', 0.35),
            ('critical_distance', 0.22),
            ('front_sector_deg', 60.0),
            ('scan_max_age', 1.0),
            ('min_valid_range', 0.12),
            ('search_angular', 0.80),
            ('closing_threshold', -0.02),
            ('use_feedforward', True),
            ('blocked_margin', 0.30),
            ('reacquire_ramp', 5.0),
            ('desired_distance_floor', 0.60),
            ('max_tf_misses', 10),
            ('odom_frame', 'odom'),
            ('base_frame', 'base_footprint'),
            ('cmd_vel_topic', '/cmd_vel'),
        ])
        g = {p.name: p.value for p in decl}

        self.d_des = float(g['desired_distance'])
        self.d_tol = float(g['distance_tolerance'])
        self.b_tol = float(g['bearing_tolerance'])
        self.k_lin = float(g['k_linear'])
        self.k_ang = float(g['k_angular'])
        self.v_max = float(g['max_linear'])
        self.w_max = float(g['max_angular'])
        self.reverse_factor = float(g['reverse_factor'])
        self.lost_timeout = float(g['lost_timeout'])
        self.use_pred = bool(g['use_prediction'])
        self.horizon = float(g['prediction_horizon'])
        self.win = int(g['velocity_window'])
        self.min_vel_samples = int(g['min_velocity_samples'])
        self.max_target_speed = float(g['max_target_speed'])
        self.v_min_pred = float(g['min_speed_for_prediction'])
        self.d_safe = float(g['safety_distance'])
        self.d_crit = float(g['critical_distance'])
        self.front_half = math.radians(float(g['front_sector_deg'])) / 2.0
        self.scan_max_age = float(g['scan_max_age'])
        self.min_range = float(g['min_valid_range'])
        self.w_search = float(g['search_angular'])
        self.closing_threshold = float(g['closing_threshold'])
        self.use_feedforward = bool(g['use_feedforward'])
        self.blocked_margin = float(g['blocked_margin'])
        self.reacquire_ramp = float(g['reacquire_ramp'])
        self.d_des_floor = float(g['desired_distance_floor'])
        self.max_tf_misses = int(g['max_tf_misses'])
        self.odom_frame = g['odom_frame']
        self.base_frame = g['base_frame']

        self.setup_state()
        self.setup_ros(g)

    def setup_state(self):
        # All the node's internal state, in one place.
        #
        # spin_thread=True is mandatory here, not an optimisation. Without it
        # the listener runs on the node's own single-threaded executor: when a
        # callback waits for a transform, that executor is blocked and cannot
        # fill the buffer, so every lookup is guaranteed to time out.
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self,
                                                     spin_thread=True)

        self.state = SEARCH
        self.last_seen = None
        self.lost_since = None
        self.last_bearing = 0.0
        self.search_dir = 1.0
        self.tf_misses = 0
        self.last_cmd = (0.0, 0.0)
        self.scan = None
        self.scan_t = None

        self.history = deque(maxlen=self.win)     # (t, x_odom, y_odom)
        self.vel_est = (0.0, 0.0)

        self.dist_hist = deque(maxlen=self.win)   # (t, distance, 0)
        self.last_pushed_t = None
        self.stamp_offset = 0.0
        self.reacquire_pending = False
        self.d_ramp_from = self.d_des
        self.d_ramp_t = None

    def setup_ros(self, g):
        # Subscriptions, publishers and the control-loop timer.
        self.create_subscription(PoseStamped, '/target/pose', self.on_pose, 10)
        self.create_subscription(LaserScan, '/scan', self.on_scan,
                                 qos_profile_sensor_data)

        self.pub_cmd = self.create_publisher(TwistStamped, g['cmd_vel_topic'], 10)
        self.pub_state = self.create_publisher(String, '/follower/state', 10)
        self.pub_derr = self.create_publisher(Float32, '/follower/distance_error', 10)
        # Camera-measured distance, published for the logger. It is the right
        # source for the accuracy metric: it is computed by whoever uses it,
        # which spares the logger from reconstructing it from poses.
        self.pub_dist = self.create_publisher(Float32, '/follower/measured_distance', 10)
        self.pub_berr = self.create_publisher(Float32, '/follower/bearing_error', 10)

        self.create_timer(1.0 / float(g['control_rate']), self.control_loop)
        self.get_logger().info(
            f"follower_controller started | "
            f"{'PREDICTIVE' if self.use_pred else 'REACTIVE'} strategy | "
            f"desired distance {self.d_des} m")

    # ------------------------------------------------------------------ input

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def capture_now(self):
        # Current time expressed on the same base as the detection timestamps,
        # so that the extrapolation interval is the elapsed prediction time and
        # nothing else.
        return self.now() - self.stamp_offset

    def on_scan(self, msg):
        self.scan = msg
        self.scan_t = self.now()

    def on_pose(self, msg):
        # The detector publishes only when it actually sees the marker, so
        # every message here is a valid detection.
        if self.last_seen is not None and (self.now() - self.last_seen) > self.lost_timeout:
            # Re-acquisition after a loss: ALL the previous history describes a
            # situation that no longer exists. Clearing only the distances and
            # not the positions left the velocity estimated over a window that
            # straddles the gap, which is a slope through two unrelated legs.
            self.history.clear()
            self.dist_hist.clear()
            self.last_pushed_t = None
            self.vel_est = (0.0, 0.0)
            self.reacquire_pending = True

        self.last_seen = self.now()

        # The MESSAGE timestamp is used, not the arrival time: it is the instant
        # the image was captured. Under a slow simulation the two differ
        # appreciably, and the difference feeds straight into the velocity
        # estimate, which is a slope with respect to time.
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if stamp <= 0.0:
            stamp = self.last_seen
        # Latency between capture and arrival. The two time bases must not be
        # mixed: the history is stamped with capture time, which is what makes
        # the velocity estimate correct, but the extrapolation interval must be
        # measured on the same base, hence this offset (see capture_now).
        self.stamp_offset = self.last_seen - stamp

        self.history.append((stamp, msg.pose.position.x, msg.pose.position.y))
        # Two physical constraints on the estimate, both needed because a
        # short window gives a wild answer: with only three samples (0.2 s) the
        # fit returns 0.25 m/s against a true 0.18, and the position predicted
        # from it runs ahead of the target. Hence a minimum number of samples
        # and a cap at what a TurtleBot3 can physically do.
        if len(self.history) < self.min_vel_samples:
            self.vel_est = (0.0, 0.0)      # not enough yet: do not extrapolate
        else:
            vx, vy = estimate_velocity(self.history)
            sp = math.hypot(vx, vy)
            if sp > self.max_target_speed:
                k = self.max_target_speed / sp
                vx, vy = vx * k, vy * k
            self.vel_est = (vx, vy)

    # ------------------------------------------------------------ measurement

    def to_base(self, px, py):
        # Take a point from the odom frame to the robot frame and return its
        # distance and bearing.
        ps = PointStamped()
        ps.header.frame_id = self.odom_frame
        ps.point.x, ps.point.y = float(px), float(py)
        try:
            pb = self.tf_buffer.transform(ps, self.base_frame)
        except Exception:
            return None
        return math.hypot(pb.point.x, pb.point.y), math.atan2(pb.point.y, pb.point.x)

    def measurement(self):
        # Target distance and bearing, extrapolated at constant velocity.
        #
        # In FOLLOW the extrapolation interval is a few hundredths of a second
        # and the correction is negligible; in PREDICT it grows and the
        # extrapolation becomes the only information available. Sharing one
        # code path means a defect in it shows up during ordinary pursuit
        # instead of hiding until the one case where it matters most.
        pred = predict_position(self.history, self.vel_est, self.capture_now(),
                                min_speed=self.v_min_pred)
        return None if pred is None else self.to_base(*pred)

    def push_distance(self, d):
        # Distance history, used to estimate its derivative. Only fresh
        # detections are recorded: the control loop runs at 20 Hz and the
        # camera at 10, so half the samples would be model extrapolations.
        if self.last_seen is not None and self.last_seen != self.last_pushed_t:
            self.last_pushed_t = self.last_seen
            self.dist_hist.append((self.last_seen, d, 0.0))

    def range_rate(self):
        # Range rate: positive when the target is moving away. Estimated with
        # the same regression used for the velocity, and for the same reason:
        # on noisy data a finite difference gives an unreliable SIGN, and here
        # only the sign is used, to decide whether reversing is allowed.
        return estimate_velocity(self.dist_hist)[0]

    # ---------------------------------------------------------------- control

    def control_loop(self):
        # The loop is split into three steps, one per responsibility: decide
        # which state to be in, produce the command for that state, filter it
        # with the LiDAR. Keeping them apart makes each readable on its own.
        self.update_state()
        v, w, d_err, b_err = self.command_for_state()
        v, w = self.arbitrate(v, w)
        self.last_cmd = (v, w)
        self.publish(v, w, d_err, b_err)

    def update_state(self):
        # The state machine transitions, and nothing else.
        lost = (self.last_seen is None
                or (self.now() - self.last_seen) > self.lost_timeout)
        if not lost:
            # Applies from ANY state: if the marker reappears, the recovery is
            # abandoned immediately.
            self.set_state(FOLLOW)
            self.lost_since = None
        elif self.state == FOLLOW:
            self.on_target_lost()
        elif self.state == PREDICT:
            self.check_prediction()

    def on_target_lost(self):
        # The ONLY point where the two strategies diverge.
        #
        # The baseline stops and looks around, as the proposal prescribes:
        if not self.use_pred:
            self.enter_search('target lost -> SEARCH (reactive baseline)')
        elif len(self.history) >= 3:
            self.lost_since = self.now()
            self.set_state(PREDICT, 'target lost -> PREDICT')
        else:
            self.enter_search('target lost -> SEARCH (too few samples)')

    def check_prediction(self):
        # The two abort criteria. The first is time; the second is a
        # contradiction between sensors, and therefore not an arbitrary
        # threshold: the camera says the target is over there, the laser says
        # there is a wall in between, and one of the two is extrapolating
        # while the other is measuring.
        if (self.now() - self.lost_since) > self.horizon:
            self.enter_search('prediction horizon expired -> SEARCH')
            return
        m = self.measurement()
        if m is not None and self.blocked_towards(*m):
            self.enter_search('predicted position is behind an obstacle -> SEARCH')

    def command_for_state(self):
        # The raw command, before the safety filter.
        if self.state not in (FOLLOW, PREDICT):
            return 0.0, self.w_search * self.search_dir, 0.0, 0.0

        m = self.measurement()
        if m is None:
            return (*self.on_tf_miss(), 0.0, 0.0)

        self.tf_misses = 0
        v, w, d_err, b_err = self.visual_command(*m)
        self.pub_dist.publish(Float32(data=float(m[0])))
        if self.state == PREDICT:
            v *= 0.7          # blind: go slower
        self.last_bearing = b_err
        return v, w, d_err, b_err

    def on_tf_miss(self):
        # NOTE: a missing measurement means the TF lookup failed, NOT that the
        # target is absent. A lookup can fail for an isolated cycle when the
        # simulation is slow; giving up at once would cut the prediction short
        # for a reason that has nothing to do with the target. The last
        # command is held for up to max_tf_misses cycles instead.
        self.tf_misses += 1
        if self.tf_misses <= self.max_tf_misses:
            return self.last_cmd
        self.enter_search('TF unavailable for too long -> SEARCH')
        return 0.0, self.w_search * self.search_dir

    def blocked_towards(self, distance, bearing):
        # Is there something between here and where the target is predicted?
        #
        # The prediction extrapolates in a straight line, so when the target
        # rounds an obstacle the estimate ends up inside or beyond it. This is
        # what detects that case, without waiting the horizon out.
        if not self.scan_is_fresh():
            return False
        free = self.range_towards(bearing)
        # Two conditions, and the first is what makes this usable. Merely
        # having something in the way is not enough: in the slalom the target
        # is lost EXACTLY because it passes behind a cylinder, so an obstacle
        # in that direction is the normal case, not the exception. Both
        # conditions together are what keep the prediction from aborting on
        # every single loss.
        return free < self.d_safe and free < distance - self.blocked_margin

    def feedforward_speed(self):
        # Component of the estimated target velocity along the line of sight.
        #
        # Without it the loop has a steady-state lag.
        if not self.use_feedforward:
            return 0.0
        pos = self.own_position()
        tgt = predict_position(self.history, self.vel_est, self.capture_now(),
                               min_speed=self.v_min_pred)
        if pos is None or tgt is None:
            return 0.0
        dx, dy = tgt[0] - pos[0], tgt[1] - pos[1]
        n = math.hypot(dx, dy)
        if n < 1e-6:
            return 0.0
        vx, vy = self.vel_est
        return (vx * dx + vy * dy) / n          # projection onto the line of sight

    def effective_setpoint(self, distance):
        # After a re-acquisition the setpoint is NOT demanded at once.
        #
        # Regaining contact from close in gives a large negative error, so the
        # setpoint ramps back to nominal instead of being demanded at once.
        if self.reacquire_pending:
            self.reacquire_pending = False
            self.d_ramp_from = min(max(distance, self.d_des_floor), self.d_des)
            self.d_ramp_t = self.now()

        if self.d_ramp_t is None:
            return self.d_des

        k = min(1.0, (self.now() - self.d_ramp_t) / max(self.reacquire_ramp, 1e-3))
        if k >= 1.0:
            self.d_ramp_t = None
        return self.d_ramp_from + (self.d_des - self.d_ramp_from) * k

    def linear_command(self, d_err):
        # Proportional action with a dead band, plus the feed-forward.
        #
        # The dead band suppresses limit cycles: without it, noise on d
        # produces a continuous alternation of small forward and backward
        # commands, visible as jitter and hard on real actuators.
        v = 0.0 if abs(d_err) < self.d_tol else self.k_lin * d_err
        v += self.feedforward_speed()

        # Reversing is gated on the SIGN of the range rate. Being too close is
        # not reason enough: if the target is already moving away the gap
        # closes by itself, and backing off would open it at the sum of both
        # speeds, pushing the marker out of detection range faster than simply
        # holding position would.
        if v < 0.0 and self.range_rate() > self.closing_threshold:
            v = 0.0
        return v

    def visual_command(self, distance, bearing):
        # Control law: two decoupled proportional loops.
        #
        # The cosine term is the only coupling between them, and it encodes
        # the non-holonomic constraint: with the target 90 degrees to the side,
        # advancing does not reduce the distance at all, so translation is
        # suppressed while the robot is misaligned. It turns first and drives
        # second. Saturating at pi/2 keeps the cosine from going negative,
        # which would command reverse as a side effect of a large bearing.
        self.push_distance(distance)
        d_err = distance - self.effective_setpoint(distance)
        v = self.linear_command(d_err)
        w = 0.0 if abs(bearing) < self.b_tol else self.k_ang * bearing
        v *= max(0.0, math.cos(min(abs(bearing), math.pi / 2)))
        return self.clamp(v, self.v_max), self.clamp(w, self.w_max), d_err, bearing

    # ----------------------------------------------------------------- safety
    #
    # Starting reference: ROBOTIS' own turtlebot3_automatic_parking package
    # (github.com/ROBOTIS-GIT/turtlebot3_applications/tree/main/
    # turtlebot3_automatic_parking, Apache License 2.0, included here as
    # LICENSE.robotis — the same package this project's detector is adapted
    # from). Its final parking step reads a fixed range of scan indices
    # (ranges[150:210], the rear sector) filtering out zero readings, and
    # checks the minimum against a single distance threshold to decide
    # whether to keep reversing or stop. Written as a minimal equivalent
    # using this project's own naming, that reference logic is:
    #
    #     def arbitrate(self, v, w):
    #         d_min, _, _ = self.sector_min(0, 30)   # <- from the reference
    #         if d_min > 0.2:                        # <- from the reference
    #             return v, w                        # clear: pass through
    #         return 0.0, 0.0                        # blocked: stop, full stop
    #
    # sector_min() below — reading a fixed range of scan indices rather
    # than computing an angle per sample — IS that reference logic, applied
    # to both the front and the rear sector. Everything past that single
    # threshold-and-stop check is this project's own addition, because on
    # its own the reference is exactly the failure mode this project's
    # documentation argues against (Section 9.1 of the report): a binary
    # go/stop response with no scaling and no notion of which side is
    # actually free. What is added: three graded regimes instead of a
    # single threshold (free / braking / stop, scaled linearly rather than
    # switched), a scan of both sides of the front sector to always turn
    # towards whichever is freer, a guaranteed way out even at zero
    # commanded speed, and the same treatment applied symmetrically to the
    # rear sector for reverse motion.

    def arbitrate(self, v, w):
        # The LiDAR does not replace the visual command, it limits it:
        #   beyond safety_distance the command passes through unchanged
        #   between critical and safety the linear velocity is scaled linearly
        #   below critical_distance only the turn away from the obstacle remains
        # Rotation is never suppressed, so the follower can keep the target in
        # frame while braking: that is what stops the safety layer from causing
        # the very loss it exists to avoid.
        if v < 0.0:
            return self.arbitrate_reverse(v, w)

        d_min, free_side = self.front_obstacle()
        if d_min >= self.d_safe:
            return v, w
        if v == 0.0:
            # stopped but close to an obstacle: the rotation towards the free
            # side is added anyway, otherwise standing still in front of a wall
            # would be a stable state with no way out
            if d_min <= self.d_crit:
                return 0.0, self.clamp(w + 0.8 * free_side, self.w_max)
            return v, w
        if d_min <= self.d_crit:
            return 0.0, self.clamp(0.8 * free_side, self.w_max)
        scale = self.brake_scale(d_min)
        return v * scale, self.clamp(w + (1.0 - scale) * 0.5 * free_side, self.w_max)

    def arbitrate_reverse(self, v, w):
        # The same graded law applied to the rear sector, by the same
        # index-based selection as front_obstacle (center at 180 degrees
        # instead of 0). The Burger's LiDAR spans a full 360 degrees, so the
        # space behind is known as well as the space ahead. Rotation is
        # never limited, so the target stays framed.
        if not self.scan_is_fresh():
            d_min = 0.0
        else:
            half_deg = int(round(math.degrees(self.front_half)))
            d_min, _, _ = self.sector_min(180, half_deg)
        v = max(v, -self.v_max * self.reverse_factor)
        if d_min >= self.d_safe:
            return v, w
        if d_min <= self.d_crit:
            return 0.0, w
        return v * self.brake_scale(d_min), w

    def brake_scale(self, d_min):
        # Braking factor between the critical and the safety distance.
        span = self.d_safe - self.d_crit
        if span <= 1e-6:
            return 0.0 if d_min <= self.d_crit else 1.0
        return max(0.0, min(1.0, (d_min - self.d_crit) / span))

    def valid_ranges(self, r):
        # Which laser returns can be trusted.
        #
        # The message's own range_min cannot be: the gz bridge delivers it as
        # zero, so filtering on it filters nothing. The LDS-01 floor of 0.12 m
        # is therefore imposed explicitly.
        lo = max(float(self.scan.range_min), self.min_range)
        return np.isfinite(r) & (r > lo) & (r < self.scan.range_max)

    def scan_is_fresh(self):
        # The safety layer must refuse to decide on stale data. If the LiDAR
        # stream stops, reusing the last scan means believing the world froze as
        # it was, and the follower would advance trusting free space that is no
        # longer known to be free.
        return (self.scan is not None and self.scan_t is not None
                and (self.now() - self.scan_t) <= self.scan_max_age)

    def freer_side(self, r, ok):
        """+1 when there is more room to the left, -1 to the right.

        Index-based, like front_obstacle: the left quarter is indices
        1..89 (1 to 89 degrees), the right quarter is 271..359 (-89 to -1
        degrees), following the same one-degree-per-sample convention.
        """
        left = np.zeros(r.size, dtype=bool)
        right = np.zeros(r.size, dtype=bool)
        left[1:90] = True
        right[271:360] = True
        left &= ok
        right &= ok
        d_l = float(np.min(r[left])) if np.any(left) else float('inf')
        d_r = float(np.min(r[right])) if np.any(right) else float('inf')
        return 1.0 if d_l >= d_r else -1.0

    def sector_min(self, center_deg, half_deg):
        # Minimum valid range within a fixed angular sector, selected by
        # INDEX rather than by computing an angle per sample.
        #
        # This is the pattern from the reference implementation named above
        # (ranges[0:60] + ranges[300:360] for a fixed front sector), rather
        # than this project's earlier angle_min + i*angle_increment
        # computation. It is exact, not an approximation, only because this
        # LiDAR is known to always return exactly 360 samples, one per
        # degree, starting at 0 (see the <lidar> block in the follower's
        # model.sdf): index i IS bearing i degrees. A sensor with a
        # different sample count or start angle would silently break this;
        # range_towards(), below, is kept independent of that assumption
        # for exactly this reason, since it must handle an arbitrary
        # bearing (the direction of a predicted, not observed, position),
        # not just this instrument's own front and rear.
        r = np.array(self.scan.ranges, dtype=np.float64)
        ok = self.valid_ranges(r)
        # +1 on the lower bound: the sector is open on both ends (matching
        # the old abs(angle) < half strict inequality), and a slice is only
        # open on its upper end, so the lower end needs the exclusion done
        # by hand. Without it, exactly the sample sitting on the boundary
        # (bearing == -half_deg exactly) would be included here but was not
        # before, e.g. index 330 for a 30-degree half-sector.
        lo, hi = (center_deg - half_deg + 1) % 360, (center_deg + half_deg) % 360
        sector = np.zeros(r.size, dtype=bool)
        if lo < hi:
            sector[lo:hi] = True
        else:                                   # wraps past 0/360, e.g. the front
            sector[lo:] = True
            sector[:hi] = True
        sector &= ok
        return float(np.min(r[sector])) if np.any(sector) else float('inf'), r, ok

    def front_obstacle(self):
        # Minimum distance in the front sector, plus the freer side.
        if not self.scan_is_fresh():
            # Stale data: assume an obstacle, which is the cautious answer.
            self.get_logger().warn('stale LiDAR scan: assuming an obstacle ahead',
                                   throttle_duration_sec=3.0)
            return 0.0, 1.0
        half_deg = int(round(math.degrees(self.front_half)))
        d_min, r, ok = self.sector_min(0, half_deg)
        return d_min, self.freer_side(r, ok)

    def range_towards(self, bearing, half_width=0.30):
        # Free distance in an ARBITRARY direction, in the robot frame —
        # used for the predicted-target bearing in blocked_towards(), which
        # can point anywhere, not just at this sensor's fixed front or rear.
        # The reference implementation this project's front-sector check is
        # based on has no equivalent of this at all: it only ever looks
        # straight ahead. This stays on angle_min/angle_increment rather
        # than on the index convention above, so it keeps working
        # regardless of how the scan is sampled.
        if not self.scan_is_fresh():
            return 0.0
        r = np.array(self.scan.ranges, dtype=np.float64)
        if r.size == 0:
            return 0.0
        ang = self.scan.angle_min + np.arange(r.size) * self.scan.angle_increment
        rel = np.arctan2(np.sin(ang - bearing), np.cos(ang - bearing))
        sel = (np.abs(rel) < half_width) & self.valid_ranges(r)
        return float(np.min(r[sel])) if np.any(sel) else float('inf')

    # --------------------------------------------------------------- recovery

    def enter_search(self, why):
        # turn towards the side where the target was last seen
        self.search_dir = 1.0 if self.last_bearing >= 0 else -1.0
        self.set_state(SEARCH, why)

    # ---------------------------------------------------------------- output

    def own_position(self):
        # Follower position in the odom frame, needed by the feed-forward to
        # project the target velocity onto the line of sight.
        try:
            t = self.tf_buffer.lookup_transform(
                self.odom_frame, self.base_frame, rclpy.time.Time())
            return (t.transform.translation.x, t.transform.translation.y)
        except Exception:
            return None

    def set_state(self, state, why=None):
        if state != self.state:
            self.state = state
            if why:
                self.get_logger().info(why)

    @staticmethod
    def clamp(x, limit):
        return max(-limit, min(limit, x))

    def publish(self, v, w, d_err, b_err):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.twist.linear.x = float(v)
        msg.twist.angular.z = float(w)
        self.pub_cmd.publish(msg)
        self.pub_state.publish(String(data=self.state))
        self.pub_derr.publish(Float32(data=float(d_err)))
        self.pub_berr.publish(Float32(data=float(b_err)))


def main():
    rclpy.init()
    node = FollowerController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # leave the robot stopped, whatever the reason for shutting down.
        # After SIGINT the context may already be torn down, and publishing on
        # a dead context raises: the stop command is best effort.
        try:
            node.publish(0.0, 0.0, 0.0, 0.0)
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
