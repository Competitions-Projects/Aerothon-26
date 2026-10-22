import math
import threading
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSProfile, ReliabilityPolicy, HistoryPolicy,
                       DurabilityPolicy, qos_profile_sensor_data)
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO

from px4_msgs.msg import (OffboardControlMode, TrajectorySetpoint,
                          VehicleCommand, VehicleLocalPosition)


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


class YoloGroundAlignmentNode(Node):
    """
    Downward-facing camera (Logitech Brio 100 -> /camera/image_raw, ~15 fps).
    Detects a green banner on the ground and holds the drone centred over it.

    Camera mounting assumption: looking straight down, image TOP = drone FRONT,
    image RIGHT = drone RIGHT. If your mount differs, flip the two *_sign params.
    """

    def __init__(self):
        super().__init__('yolo_ground_alignment_node')

        # ---------------- Parameters ----------------
        self.declare_parameter('model_path', 'bestgz.onnx')   # use a .engine on the Jetson if you can
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('conf', 0.5)
        self.declare_parameter('imgsz', 640)
        self.declare_parameter('target_altitude', 3.0)        # metres above ground (positive number)
        self.declare_parameter('kp', 0.8)                     # m/s per unit normalised error (-1..1)
        self.declare_parameter('kd', 0.15)
        self.declare_parameter('max_speed', 0.5)              # m/s lateral limit
        self.declare_parameter('lock_tol', 0.08)              # normalised error considered "centred"
        self.declare_parameter('lock_frames', 8)              # consecutive centred frames for LOCK
        self.declare_parameter('forward_sign', 1.0)           # flip to -1.0 if forward/back is reversed
        self.declare_parameter('right_sign', 1.0)             # flip to -1.0 if left/right is reversed
        self.declare_parameter('detect_timeout', 0.6)         # s without detection -> stop moving
        self.declare_parameter('show_debug', False)           # keep False when running headless

        # Detector: 'hsv' (plain solid green banner, no training needed) or 'yolo'
        self.declare_parameter('detector', 'hsv')
        # Green range in OpenCV HSV (H 0-179, S/V 0-255). Tune with the organisers' RGB values.
        self.declare_parameter('h_low', 35)
        self.declare_parameter('h_high', 85)
        self.declare_parameter('s_min', 70)
        self.declare_parameter('v_min', 50)
        self.declare_parameter('min_area', 0.03)              # min blob size as fraction of the frame
        self.declare_parameter('min_fill', 0.70)              # blob area / its rotated bounding rectangle (banner is a rectangle)
        # Optional: require the ~3 inch white border around the green blob (rejects grass etc.)
        self.declare_parameter('border_check', False)
        self.declare_parameter('border_px', 10)               # border width in px on the 320 px-wide working image
        self.declare_parameter('border_frac', 0.30)           # fraction of the ring that must be white
        self.declare_parameter('white_s_max', 60)
        self.declare_parameter('white_v_min', 170)

        p = self.get_parameter
        self.conf = p('conf').value
        self.imgsz = p('imgsz').value
        self.target_altitude = -abs(p('target_altitude').value)   # NED: up is negative
        self.kp = p('kp').value
        self.kd = p('kd').value
        self.max_speed = p('max_speed').value
        self.lock_tol = p('lock_tol').value
        self.lock_frames = p('lock_frames').value
        self.forward_sign = p('forward_sign').value
        self.right_sign = p('right_sign').value
        self.detect_timeout = p('detect_timeout').value
        self.show_debug = p('show_debug').value
        self.detector = p('detector').value
        self.h_low = p('h_low').value
        self.h_high = p('h_high').value
        self.s_min = p('s_min').value
        self.v_min = p('v_min').value
        self.min_area = p('min_area').value
        self.min_fill = p('min_fill').value
        self.border_check = p('border_check').value
        self.border_px = p('border_px').value
        self.border_frac = p('border_frac').value
        self.white_s_max = p('white_s_max').value
        self.white_v_min = p('white_v_min').value

        # ---------------- QoS ----------------
        pub_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             history=HistoryPolicy.KEEP_LAST, depth=1)
        sub_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                             durability=DurabilityPolicy.VOLATILE,
                             history=HistoryPolicy.KEEP_LAST, depth=1)

        # ---------------- Subscribers ----------------
        self.create_subscription(Image, p('image_topic').value,
                                 self.image_callback, qos_profile_sensor_data)
        # NOTE: VehicleLocalPosition lives on vehicle_local_position, not vehicle_odometry.
        # (On newer PX4 versions the topic may be versioned, e.g. /fmu/out/vehicle_local_position_v1.)
        self.create_subscription(VehicleLocalPosition, '/fmu/out/vehicle_local_position',
                                 self.position_callback, sub_qos)

        # ---------------- Publishers ----------------
        self.cmd_pub = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', pub_qos)
        self.mode_pub = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', pub_qos)
        self.setpoint_pub = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', pub_qos)

        # ---------------- Model ----------------
        self.model = None
        if self.detector == 'yolo':
            model_path = p('model_path').value
            self.get_logger().info(f"Loading YOLO model: {model_path}")
            self.model = YOLO(model_path, task='detect')
            self.get_logger().info("Model loaded.")
        else:
            self.get_logger().info("Using HSV green-banner detector.")

        self.bridge = CvBridge()

        # ---------------- State ----------------
        self.current_z = 0.0
        self.heading = 0.0
        self.flight_phase = "TAKEOFF"        # TAKEOFF -> TRACKING
        self.counter = 0
        self.locked = False
        self.aligned_counter = 0

        # Commands produced by the vision thread (body frame), read by the timer
        self.cmd_lock = threading.Lock()
        self.cmd_forward = 0.0
        self.cmd_right = 0.0
        self.last_detect_time = 0.0

        # PD memory
        self.prev_ex = 0.0
        self.prev_ey = 0.0
        self.prev_t = None
        self.d_ex = 0.0
        self.d_ey = 0.0

        # Latest-frame mailbox: the camera callback only stores the frame, so
        # slow inference can never block the 20 Hz offboard heartbeat.
        self.frame_lock = threading.Lock()
        self.latest_msg = None
        self.frame_event = threading.Event()
        self.running = True
        self.worker = threading.Thread(target=self.vision_loop, daemon=True)
        self.worker.start()

        self.timer = self.create_timer(0.05, self.timer_callback)   # 20 Hz

    # ------------------------------------------------------------------
    def position_callback(self, msg):
        self.current_z = msg.z
        if math.isfinite(msg.heading):
            self.heading = msg.heading

    def publish_vehicle_command(self, command, param1=0.0, param2=0.0):
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = float(param1)
        msg.param2 = float(param2)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.cmd_pub.publish(msg)

    # ------------------------------------------------------------------
    def image_callback(self, msg):
        if self.flight_phase != "TRACKING":
            return
        with self.frame_lock:
            self.latest_msg = msg          # overwrite: always process the newest frame
        self.frame_event.set()

    def vision_loop(self):
        while self.running:
            if not self.frame_event.wait(0.1):
                continue
            self.frame_event.clear()
            with self.frame_lock:
                msg, self.latest_msg = self.latest_msg, None
            if msg is None:
                continue
            try:
                self.process_frame(msg)
            except Exception as e:
                self.get_logger().error(f"Vision error: {e}")

    def process_frame(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        h, w, _ = frame.shape
        if self.detector == 'yolo':
            det, debug = self.detect_yolo(frame)
        else:
            det, debug = self.detect_hsv(frame)

        if det is None:
            self.aligned_counter = 0
            self.prev_t = None             # avoid a bogus derivative on re-acquire
            self.show(debug)
            return

        cx, cy = det                       # banner centre in full-frame pixels

        # Normalised error: -1..1, resolution independent.
        # ex > 0: banner is right of centre.  ey > 0: banner is below centre (behind the drone).
        ex = (cx - w / 2.0) / (w / 2.0)
        ey = (cy - h / 2.0) / (h / 2.0)

        now = time.monotonic()
        if self.prev_t is None:
            dex = dey = 0.0
        else:
            dt = max(now - self.prev_t, 0.02)
            dex = (ex - self.prev_ex) / dt
            dey = (ey - self.prev_ey) / dt
        self.prev_t, self.prev_ex, self.prev_ey = now, ex, ey

        # Low-pass the derivative (15 fps is noisy), then clamp
        a = 0.5
        self.d_ex = a * dex + (1 - a) * self.d_ex
        self.d_ey = a * dey + (1 - a) * self.d_ey
        d_ex = clamp(self.d_ex, -2.0, 2.0)
        d_ey = clamp(self.d_ey, -2.0, 2.0)

        right = self.right_sign * (self.kp * ex + self.kd * d_ex)
        forward = self.forward_sign * -(self.kp * ey + self.kd * d_ey)  # banner above centre -> go forward

        with self.cmd_lock:
            self.cmd_right = clamp(right, -self.max_speed, self.max_speed)
            self.cmd_forward = clamp(forward, -self.max_speed, self.max_speed)
            self.last_detect_time = now

        # Lock detection (informational; tracking keeps running to hold position)
        if abs(ex) < self.lock_tol and abs(ey) < self.lock_tol:
            self.aligned_counter += 1
            if self.aligned_counter >= self.lock_frames and not self.locked:
                self.locked = True
                self.get_logger().info("LOCK: drone is centred over the banner.")
        else:
            self.aligned_counter = max(0, self.aligned_counter - 1)
            if self.aligned_counter == 0:
                self.locked = False

        self.show(debug)

    # ---------------- Detectors: return ((cx, cy) or None, debug_image) ----------------
    def detect_yolo(self, frame):
        results = self.model(frame, conf=self.conf, imgsz=self.imgsz, verbose=False)
        boxes = results[0].boxes
        debug = results[0].plot() if self.show_debug else frame
        if len(boxes) == 0:
            return None, debug
        best = max(boxes, key=lambda b: float(b.conf[0]))
        x1, y1, x2, y2 = best.xyxy[0].cpu().numpy()
        return ((x1 + x2) / 2.0, (y1 + y2) / 2.0), debug

    def detect_hsv(self, frame):
        h, w = frame.shape[:2]
        scale = 320.0 / w if w > 320 else 1.0            # work small: fast on the Orin Nano
        small = cv2.resize(frame, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA) if scale < 1.0 else frame
        sh, sw = small.shape[:2]

        hsv = cv2.cvtColor(cv2.GaussianBlur(small, (5, 5), 0), cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, np.array([self.h_low, self.s_min, self.v_min]),
                           np.array([self.h_high, 255, 255]))
        k = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, k)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        best, best_area = None, 0.0
        for c in contours:
            area = cv2.contourArea(c)
            if area < self.min_area * sw * sh or area <= best_area:
                continue
            rw, rh = cv2.minAreaRect(c)[1]
            if rw * rh <= 0 or area / (rw * rh) < self.min_fill:     # banner must be rectangular
                continue
            if self.border_check and not self.has_white_border(hsv, c, mask.shape):
                continue
            best, best_area = c, area

        debug = frame
        if self.show_debug:
            debug = frame.copy()
            if best is not None:
                cv2.drawContours(debug, [(best / scale).astype(np.int32)], -1, (0, 0, 255), 2)

        if best is None:
            return None, debug
        m = cv2.moments(best)
        if m['m00'] <= 0:
            return None, debug
        return (m['m10'] / m['m00'] / scale, m['m01'] / m['m00'] / scale), debug

    def has_white_border(self, hsv, contour, shape):
        filled = np.zeros(shape, np.uint8)
        cv2.drawContours(filled, [contour], -1, 255, -1)
        grown = cv2.dilate(filled, np.ones((2 * self.border_px + 1, 2 * self.border_px + 1), np.uint8))
        ring = cv2.subtract(grown, filled)
        n = cv2.countNonZero(ring)
        if n == 0:
            return False
        white = cv2.inRange(hsv, np.array([0, 0, self.white_v_min]),
                            np.array([179, self.white_s_max, 255]))
        return cv2.countNonZero(cv2.bitwise_and(white, ring)) / n >= self.border_frac

    def show(self, img):
        if not self.show_debug:
            return
        try:
            cv2.imshow("Drone Camera", img)
            cv2.waitKey(1)
        except cv2.error:
            pass

    # ------------------------------------------------------------------
    def timer_callback(self):
        timestamp = int(self.get_clock().now().nanoseconds / 1000)

        offboard_msg = OffboardControlMode()
        offboard_msg.position = False
        offboard_msg.velocity = True
        offboard_msg.timestamp = timestamp
        self.mode_pub.publish(offboard_msg)

        forward = right = 0.0
        vz = 0.0

        if self.counter > 10:
            if self.flight_phase == "TAKEOFF":
                vz = -0.8
                if self.current_z <= self.target_altitude + 0.2:
                    self.flight_phase = "TRACKING"
                    self.get_logger().info("Altitude reached. Tracking ground banner.")

            elif self.flight_phase == "TRACKING":
                with self.cmd_lock:
                    fresh = (time.monotonic() - self.last_detect_time) < self.detect_timeout
                    if fresh:
                        forward, right = self.cmd_forward, self.cmd_right
                # Not fresh -> forward = right = 0 -> drone just holds position
                alt_error = self.target_altitude - self.current_z
                vz = clamp(0.5 * alt_error, -0.5, 0.5)

        # Body (forward/right) -> NED (north/east) using current heading
        c, s = math.cos(self.heading), math.sin(self.heading)
        vn = forward * c - right * s
        ve = forward * s + right * c

        traj = TrajectorySetpoint()
        traj.position = [float('nan')] * 3
        traj.velocity = [float(vn), float(ve), float(vz)]
        traj.yaw = float('nan')
        traj.yawspeed = 0.0
        traj.timestamp = timestamp
        self.setpoint_pub.publish(traj)

        if self.counter >= 10 and self.counter % 10 == 0:
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)

        self.counter += 1

    def shutdown(self):
        self.running = False
        self.frame_event.set()


def main(args=None):
    rclpy.init(args=args)
    node = YoloGroundAlignmentNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
