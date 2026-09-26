# ArUco detection and pose estimation for the target robot.
#
# Adapted from ROBOTIS turtlebot3_aruco_tracker
# (https://github.com/ROBOTIS-GIT/turtlebot3_applications/tree/main/turtlebot3_aruco_tracker),
# Apache License 2.0, Copyright 2025 ROBOTIS CO., LTD., Author: ChanHyeong Lee.
#
# What is kept from the original: the overall node shape (one CameraInfo
# subscription consumed once, one Image subscription driving detection), the
# camera_info consume-once-then-unsubscribe pattern, and the detection loop
# shape (enumerate over the detected marker IDs).
#
# What is changed, and why:
#   - Image, not CompressedImage: our camera bridge (ros_gz_image) publishes
#     raw frames, the original targets a compressed topic from real hardware.
#   - The marker dictionary is kept at DICT_4X4_50 (the original uses
#     DICT_5X5_250) to match the markers already printed into this project's
#     arena (see worlds.py); changing the dictionary would mean re-texturing
#     every panel for no benefit.
#   - Detection calls the OpenCV ArUco API directly, pinned to the classic
#     (<=4.6) free-function form: Ubuntu 24.04 / ROS Jazzy ships OpenCV
#     4.6.0 via apt, where cv2.aruco.ArucoDetector does not exist yet, so
#     there is exactly one API to call, not two to support.
#   - An identifier filter was added: four markers of the same dictionary
#     exist in this arena (three fixed panels plus the target), and the
#     original — built for a single-marker parking scenario — has no notion
#     of "the marker I actually care about". Without it the follower would
#     react to whichever panel the camera saw first.
#   - The pose is published as a PoseStamped on /target/pose, transformed
#     into the fixed odom frame, rather than broadcast as a TF. The rest of
#     this project (velocity estimation, the control law) already expects a
#     topic in a fixed frame; building that on a TF broadcast would mean
#     every consumer performs its own lookup for no benefit.
#   - Orientation is left at identity instead of the corrected quaternion
#     the original computes (which needs scipy, an extra dependency this
#     project does not otherwise have): nothing downstream reads it, the
#     controller works from position alone (distance and bearing).
import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Bool, Int32
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker, MarkerArray
from cv_bridge import CvBridge
import tf2_ros
import tf2_geometry_msgs  # noqa: F401  registers the PoseStamped transform
from geometry_msgs.msg import TransformStamped


class TargetDetector(Node):

    def __init__(self):
        super().__init__('target_detector_node')

        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/camera_info')
        # side of the marker's BLACK SQUARE, not of the white panel that
        # contains it: the quiet zone is not part of what solvePnP solves for,
        # and confusing the two biases every distance by their ratio
        self.declare_parameter('marker_size', 0.15)
        self.declare_parameter('target_marker_id', 3)
        self.declare_parameter('optical_frame', 'camera_rgb_optical_frame')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('publish_debug_image', True)
        # marker frame on /tf, for RViz only: no node performs lookups on it.
        # It is intermittent, appearing and disappearing with the detection,
        # which is acceptable while it stays purely visual.
        self.declare_parameter('publish_marker_tf', True)

        self.marker_size = float(self.get_parameter('marker_size').value)
        self.target_id = int(self.get_parameter('target_marker_id').value)
        self.optical_frame = self.get_parameter('optical_frame').value
        self.odom_frame = self.get_parameter('odom_frame').value
        debug = bool(self.get_parameter('publish_debug_image').value)
        self.publish_tf = bool(self.get_parameter('publish_marker_tf').value)

        # DICT_4X4_50 kept explicitly (see module header): the upstream
        # tracker defaults to DICT_5X5_250.
        self.dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.aruco_params = cv2.aruco.DetectorParameters_create()
        # Sub-pixel corner refinement: the pose depends directly on the
        # corner positions, so refining them measurably reduces the noise on
        # the estimated distance and bearing.
        self.aruco_params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        self.bridge = CvBridge()
        self.K = None
        self.D = None

        self.tf_buffer = tf2_ros.Buffer()
        # spin_thread=True: without it the listener runs on the node's own
        # executor, and the blocking lookup inside the image callback prevents
        # the buffer from filling, guaranteeing a full timeout on every miss.
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self,
                                                     spin_thread=True)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self) if self.publish_tf else None
        self.marker_frame = f'aruco_marker_{self.target_id}'

        # Pattern from the ROBOTIS tracker: consume CameraInfo once, then
        # drop the subscription entirely, rather than keep receiving and
        # discarding messages on every frame (intrinsics are constant here).
        self.sub_info = self.create_subscription(
            CameraInfo, self.get_parameter('camera_info_topic').value,
            self.on_info, qos_profile_sensor_data)
        self.create_subscription(Image, self.get_parameter('image_topic').value,
                                 self.on_image, qos_profile_sensor_data)

        self.pub_pose = self.create_publisher(PoseStamped, '/target/pose', 10)
        # The proposal requires the detection node to publish the marker ID
        # and a visibility flag as well. The controller uses neither: the
        # silence of the pose topic is enough, because the detector publishes
        # only on a successful detection. They are published anyway, for
        # inspection with ros2 topic echo and to honour the interface.
        self.pub_visible = self.create_publisher(Bool, '/target/visible', 10)
        self.pub_id = self.create_publisher(Int32, '/target/marker_id', 10)
        self.pub_viz = self.create_publisher(MarkerArray, '/target/markers', 10)
        self.pub_debug = self.create_publisher(Image, '/target/debug_image', 1) if debug else None

        self.get_logger().info(
            f"target_detector started | id={self.target_id} "
            f"marker_size={self.marker_size} m | API "
            f"{'new' if self.new_api else 'classic (<=4.6)'}")

    def on_info(self, msg):
        self.K = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        self.D = np.array(msg.d, dtype=np.float64).reshape(1, -1)
        self.get_logger().info(f"CameraInfo received | fx={self.K[0, 0]:.1f} px")
        self.destroy_subscription(self.sub_info)

    def on_image(self, msg):
        # The callback is split by responsibility: convert, search, publish,
        # draw. Each one reads on its own.
        if self.K is None:
            return
        frame = self.to_cv(msg)
        if frame is None:
            return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = cv2.aruco.detectMarkers(
            gray, self.dictionary, parameters=self.aruco_params)
        found = self.tracking_markers(corners, ids, msg)

        # The flag is published ALWAYS, even with no marker in sight: it is the
        # only topic that states non-visibility explicitly.
        self.pub_visible.publish(Bool(data=found))
        self.publish_debug(frame, corners, ids, found, msg)

    def to_cv(self, msg):
        # A malformed frame must not bring the node down: skip it.
        try:
            return self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().warn(f"frame could not be converted: {e}",
                                   throttle_duration_sec=5.0)
            return None

    def tracking_markers(self, corners, ids, msg):
        # Loop shape from the upstream tracker's tracking_markers(): iterate
        # detected markers with enumerate(ids.flatten()). Unlike the
        # original, which has no notion of "the marker I care about" and
        # reacts to every detection, this filters by identifier: four
        # markers of the same dictionary exist in the arena (three fixed
        # panels and the target), and more than one is often visible in a
        # single frame. The main trajectory deliberately passes alongside
        # all three fixed panels so that this filter can be seen working.
        if ids is None:
            return False
        for i, marker_id in enumerate(ids.flatten()):
            if int(marker_id) != self.target_id:
                continue
            return self.publish_target(corners[i], msg)
        return False

    def publish_target(self, corner, msg):
        # From the pose in the optical frame to the pose in the fixed frame.
        #
        # The transformation happens HERE and not in the controller, so the
        # velocity estimate downstream is free of the follower's own motion.
        _, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
            [corner], self.marker_size, self.K, self.D)
        x, y, z = (float(v) for v in tvecs[0][0])

        ps = PoseStamped()
        ps.header.frame_id = self.optical_frame
        ps.header.stamp = msg.header.stamp      # CAPTURE instant
        ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = x, y, z
        ps.pose.orientation.w = 1.0
        self.broadcast_marker_tf(x, y, z, msg.header.stamp)

        try:
            po = self.tf_buffer.transform(ps, self.odom_frame,
                                          timeout=Duration(seconds=0.15))
        except Exception as e:
            self.get_logger().warn(f"TF to {self.odom_frame} not ready: {e}",
                                   throttle_duration_sec=3.0)
            return False

        po.pose.position.z = 0.0                # the motion is planar
        self.pub_pose.publish(po)
        self.pub_id.publish(Int32(data=int(self.target_id)))
        self.publish_viz(po, msg.header.stamp)
        self.get_logger().info(
            f"target @ odom ({po.pose.position.x:.2f}, "
            f"{po.pose.position.y:.2f}) | d={np.linalg.norm([x, y, z]):.2f} m",
            throttle_duration_sec=2.0)
        return True

    def publish_debug(self, frame, corners, ids, found, msg):
        # Drawing and re-encoding costs on every frame, so it is done only
        # when something is actually subscribed (RViz or rqt_image_view).
        if self.pub_debug is None or self.pub_debug.get_subscription_count() == 0:
            return
        if ids is not None:
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)
        txt = f"TARGET {self.target_id} OK" if found else "TARGET LOST"
        color = (0, 200, 0) if found else (0, 0, 255)
        cv2.putText(frame, txt, (10, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
        out = self.bridge.cv2_to_imgmsg(frame, 'bgr8')
        out.header = msg.header
        self.pub_debug.publish(out)

    def broadcast_marker_tf(self, x, y, z, stamp):
        # Broadcast the marker frame as a child of the optical frame, so the
        # detection can be seen appearing and disappearing in RViz. Publishing
        # an intermittent frame is normally poor practice, because consumers
        # cannot tell a missing frame from a stale one; it is acceptable here
        # only because no node ever performs a lookup on it.
        if self.tf_broadcaster is None:
            return
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = self.optical_frame
        t.child_frame_id = self.marker_frame
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = z
        t.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(t)

    def publish_viz(self, pose_odom, stamp):
        arr = MarkerArray()

        m = Marker()
        m.header.frame_id = self.odom_frame
        m.header.stamp = stamp
        m.ns = 'target'
        m.id = self.target_id
        m.type = Marker.SPHERE
        m.action = Marker.ADD
        m.pose = pose_odom.pose
        m.scale.x = m.scale.y = m.scale.z = 0.15
        m.color.g = 1.0
        m.color.a = 0.9
        arr.markers.append(m)

        t = Marker()
        t.header.frame_id = self.odom_frame
        t.header.stamp = stamp
        t.ns = 'target_id'
        t.id = self.target_id + 1000
        t.type = Marker.TEXT_VIEW_FACING
        t.action = Marker.ADD
        t.pose = pose_odom.pose
        t.pose.position.z += 0.35
        t.scale.z = 0.18
        t.color.r = t.color.g = t.color.b = t.color.a = 1.0
        t.text = f"ID {self.target_id}"
        arr.markers.append(t)

        self.pub_viz.publish(arr)


def main(args=None):
    rclpy.init(args=args)
    node = TargetDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
