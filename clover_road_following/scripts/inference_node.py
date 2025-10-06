#!/usr/bin/env python3
import os
import threading
import numpy as np
import rospy
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist, TwistStamped, PoseStamped
from std_srvs.srv import SetBool, SetBoolResponse, SetBoolRequest

try:
    import onnxruntime as ort
except Exception as e:
    rospy.logerr("Failed to import onnxruntime: %s", e)
    raise


class RoadFollowingNode:
    def __init__(self):
        # Core params
        self.model_path = rospy.get_param("~model_path", os.path.expanduser("~/models/model_best.onnx"))
        self.image_topic = rospy.get_param("~image_topic", "/main_camera/image_raw")
        self.cmd_pub_topic = rospy.get_param("~cmd_pub_topic", "/mavros/setpoint_velocity/cmd_vel_unstamped")
        self.fwd_speed = float(rospy.get_param("~fwd_speed", 0.3))
        self.max_yaw_rate = float(rospy.get_param("~max_yaw_rate", 0.8))
        self.alpha = float(rospy.get_param("~smooth_alpha", 0.3))  # EMA smoothing yaw
        self.camera_rotate_deg = float(rospy.get_param("~camera_rotate_deg", 0.0))  # 0/90/180/270
        self.min_edge_density = float(rospy.get_param("~min_edge_density", 0.02))
        self.publish_rate_hz = float(rospy.get_param("~publish_rate_hz", 20.0))
        self.enable_confidence_stop = bool(rospy.get_param("~enable_confidence_stop", True))
        self.cmd_msg_type = rospy.get_param("~cmd_msg_type", "auto")  # "auto" | "twist" | "stamped"

        # Altitude-hold params (PD on z-velocity)
        self.hold_altitude = bool(rospy.get_param("~hold_altitude", True))
        self.alt_mode = rospy.get_param("~alt_mode", "capture")  # "capture" | "fixed"
        self.alt_target_z = float(rospy.get_param("~alt_target_z", 1.5))  # m (ENU up)
        self.kp_z = float(rospy.get_param("~kp_z", 1.0))
        self.kd_z = float(rospy.get_param("~kd_z", 0.6))
        self.max_z_vel = float(rospy.get_param("~max_z_vel", 0.5))  # m/s limit on vz
        self.invert_z_sign = bool(rospy.get_param("~invert_z_sign", False))  # set True if sim expects opposite sign

        # Bridge / buffers
        self.bridge = CvBridge()
        self._img_lock = threading.Lock()
        self._last_bgr = None
        self._last_stamp = rospy.Time(0)

        # Altitude state
        self._z_lock = threading.Lock()
        self._z = None
        self._z_prev = None
        self._z_t = None
        self._z_t_prev = None

        # Load ONNX model (CPU-only to avoid CUDA deps)
        providers = ["CPUExecutionProvider"]
        try:
            self.session = ort.InferenceSession(self.model_path, providers=providers)
        except Exception as e:
            rospy.logerr("Failed to load ONNX model at %s: %s", self.model_path, e)
            raise

        # Determine input tensor info
        self.input_name = self.session.get_inputs()[0].name
        in_shape = self.session.get_inputs()[0].shape  # [N,C,H,W] possibly dynamic
        self.in_h, self.in_w = 90, 160
        if len(in_shape) == 4:
            h, w = in_shape[2], in_shape[3]
            if isinstance(h, int) and h > 0:
                self.in_h = h
            if isinstance(w, int) and w > 0:
                self.in_w = w

        # Decide publisher message type
        if self.cmd_msg_type == "twist":
            self.use_twist = True
        elif self.cmd_msg_type == "stamped":
            self.use_twist = False
        else:
            self.use_twist = self.cmd_pub_topic.endswith("_unstamped")

        if self.use_twist:
            self.cmd_pub = rospy.Publisher(self.cmd_pub_topic, Twist, queue_size=10)
            pub_type = "Twist"
        else:
            self.cmd_pub = rospy.Publisher(self.cmd_pub_topic, TwistStamped, queue_size=10)
            pub_type = "TwistStamped"

        # Subscriptions and services
        self.enabled = True  # keep stream alive by default
        rospy.Subscriber(self.image_topic, Image, self.image_cb, queue_size=1, buff_size=2**24)
        rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self.pose_cb, queue_size=10)
        self.toggle_srv = rospy.Service("~toggle", SetBool, self.toggle_cb)

        # Yaw smoothing state
        self._yaw_smooth = 0.0
        self._init_smooth = True

        # Logs
        provs = []
        try:
            provs = ort.get_available_providers()
        except Exception:
            pass
        rospy.loginfo("[road_following] Loaded model: %s", self.model_path)
        rospy.loginfo("[road_following] Providers: %s", provs)
        rospy.loginfo("[road_following] Input size: %dx%d", self.in_w, self.in_h)
        rospy.loginfo("[road_following] Publishing: %s (%s) at %.1f Hz", self.cmd_pub_topic, pub_type, self.publish_rate_hz)
        rospy.loginfo("[road_following] Params: fwd=%.2f, max_yaw=%.2f, alpha=%.2f, rotate=%.1fdeg, conf_stop=%s, min_edge=%.3f, hold_alt=%s (%s, z=%.2f), kp_z=%.2f kd_z=%.2f max_vz=%.2f invz=%s",
                      self.fwd_speed, self.max_yaw_rate, self.alpha, self.camera_rotate_deg,
                      str(self.enable_confidence_stop), self.min_edge_density,
                      str(self.hold_altitude), self.alt_mode, self.alt_target_z,
                      self.kp_z, self.kd_z, self.max_z_vel, str(self.invert_z_sign))

    def toggle_cb(self, req: SetBoolRequest):
        self.enabled = bool(req.data)
        # Capture altitude ref on enable if requested
        if self.enabled and self.hold_altitude and self.alt_mode.lower() == "capture":
            with self._z_lock:
                if self._z is not None:
                    self.alt_target_z = float(self._z)
                    rospy.loginfo("[road_following] Captured alt_target_z=%.3f m (ENU)", self.alt_target_z)
                else:
                    rospy.logwarn("[road_following] Cannot capture altitude target: z is None")
        msg = "enabled" if self.enabled else "disabled"
        rospy.loginfo("[road_following] Toggled: %s", msg)
        return SetBoolResponse(success=True, message=msg)

    def image_cb(self, msg: Image):
        try:
            bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            rospy.logwarn_throttle(2.0, "[road_following] cv_bridge error: %s", e)
            return
        with self._img_lock:
            self._last_bgr = bgr
            self._last_stamp = msg.header.stamp if msg.header.stamp and msg.header.stamp.to_sec() > 0 else rospy.Time.now()

    def pose_cb(self, msg: PoseStamped):
        z = float(msg.pose.position.z)  # ENU up
        t = msg.header.stamp if msg.header.stamp and msg.header.stamp.to_sec() > 0 else rospy.Time.now()
        with self._z_lock:
            self._z_prev = self._z
            self._z_t_prev = self._z_t
            self._z = z
            self._z_t = t

    @staticmethod
    def _rotate_bgr(bgr: np.ndarray, deg: float) -> np.ndarray:
        if bgr is None:
            return None
        d = int(deg) % 360
        if d == 0:
            return bgr
        elif d == 90:
            return cv2.rotate(bgr, cv2.ROTATE_90_CLOCKWISE)
        elif d == 180:
            return cv2.rotate(bgr, cv2.ROTATE_180)
        elif d == 270:
            return cv2.rotate(bgr, cv2.ROTATE_90_COUNTERCLOCKWISE)
        else:
            h, w = bgr.shape[:2]
            M = cv2.getRotationMatrix2D((w/2, h/2), d, 1.0)
            return cv2.warpAffine(bgr, M, (w, h), flags=cv2.INTER_LINEAR)

    def _preprocess(self, bgr: np.ndarray) -> np.ndarray:
        bgr = self._rotate_bgr(bgr, self.camera_rotate_deg)
        resized = cv2.resize(bgr, (self.in_w, self.in_h), interpolation=cv2.INTER_LINEAR)
        rgb = cv2.cvtColor(resized, cv2.COLOR_BGR2RGB).astype(np.float32) / 255.0
        chw = np.transpose(rgb, (2, 0, 1))
        blob = np.expand_dims(chw, axis=0).astype(np.float32).copy()
        return blob, resized

    def _infer_yaw(self, blob: np.ndarray) -> float:
        outs = self.session.run(None, {self.input_name: blob})
        y = outs[0]
        yaw = float(y.reshape(-1)[0])
        return yaw

    def _edge_confidence_ok(self, resized_bgr: np.ndarray) -> bool:
        if not self.enable_confidence_stop:
            return True
        gray = cv2.cvtColor(resized_bgr, cv2.COLOR_BGR2GRAY)
        edges = cv2.Canny(gray, 50, 150)
        density = float(np.count_nonzero(edges)) / float(edges.size)
        return density >= self.min_edge_density

    def _smooth_yaw(self, y: float) -> float:
        if self._init_smooth:
            self._yaw_smooth = y
            self._init_smooth = False
        else:
            self._yaw_smooth = self.alpha * y + (1.0 - self.alpha) * self._yaw_smooth
        return self._yaw_smooth

    def _clamp(self, v: float, lo: float, hi: float) -> float:
        return max(lo, min(hi, v))

    def _compute_vz_hold(self) -> float:
        if not self.hold_altitude:
            return 0.0
        with self._z_lock:
            if self._z is None or self._z_t is None or self._z_prev is None or self._z_t_prev is None:
                return 0.0
            z = self._z
            t = self._z_t.to_sec()
            z_prev = self._z_prev
            t_prev = self._z_t_prev.to_sec()
        dt = t - t_prev if t_prev is not None else 0.0
        dz = (z - z_prev) / dt if dt and dt > 1e-3 else 0.0
        err = self.alt_target_z - z  # ENU up
        vz = self.kp_z * err - self.kd_z * dz  # PD
        if self.invert_z_sign:
            vz = -vz
        return self._clamp(vz, -self.max_z_vel, self.max_z_vel)

    def _publish_cmd(self, vx: float, yaw_rate: float, vz: float, stamp: rospy.Time):
        if self.use_twist:
            msg = Twist()
            msg.linear.x = vx
            msg.linear.y = 0.0
            msg.linear.z = vz
            msg.angular.x = 0.0
            msg.angular.y = 0.0
            msg.angular.z = yaw_rate
            self.cmd_pub.publish(msg)
        else:
            ts = TwistStamped()
            ts.header.stamp = stamp if stamp and stamp.to_sec() > 0 else rospy.Time.now()
            ts.header.frame_id = ""
            ts.twist.linear.x = vx
            ts.twist.linear.y = 0.0
            ts.twist.linear.z = vz
            ts.twist.angular.x = 0.0
            ts.twist.angular.y = 0.0
            ts.twist.angular.z = yaw_rate
            self.cmd_pub.publish(ts)

    def spin(self):
        rate = rospy.Rate(self.publish_rate_hz)
        while not rospy.is_shutdown():
            # Copy latest image under lock
            with self._img_lock:
                bgr = None if self._last_bgr is None else self._last_bgr.copy()
                stamp = self._last_stamp

            # Compute vz regardless of enabled state to keep altitude when enabled
            vz_cmd = self._compute_vz_hold() if self.enabled else 0.0

            if bgr is not None:
                blob, resized = self._preprocess(bgr)
                if self.enabled:
                    try:
                        yaw_pred = self._infer_yaw(blob)
                    except Exception as e:
                        rospy.logwarn_throttle(2.0, "[road_following] Inference error: %s", e)
                        yaw_pred = 0.0

                    yaw_sm = self._smooth_yaw(yaw_pred)
                    yaw_out = self._clamp(yaw_sm, -self.max_yaw_rate, self.max_yaw_rate)

                    vx = self.fwd_speed
                    if self.enable_confidence_stop and not self._edge_confidence_ok(resized):
                        vx = 0.0  # stop forward when low confidence (still hold altitude via vz_cmd)

                    self._publish_cmd(vx, yaw_out, vz_cmd, stamp)
                else:
                    # Disabled: keep stream alive with zero XY & yaw; no altitude hold
                    self._publish_cmd(0.0, 0.0, 0.0, rospy.Time.now())
            else:
                # No image yet: keep stream alive with zeros
                self._publish_cmd(0.0, 0.0, 0.0, rospy.Time.now())

            rate.sleep()


if __name__ == "__main__":
    rospy.init_node("road_following")
    node = RoadFollowingNode()
    node.spin()