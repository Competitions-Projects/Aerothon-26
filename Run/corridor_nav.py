#!/usr/bin/env python3
"""
PX4 offboard corridor leg: fly forward at 5 m, file the target QR, drop to 1.75 m at the corridor entrance,
thread the 5 ft corridor, stop at its end.

    INIT -> TAKEOFF -> CRUISE -> ENTRY -> CORRIDOR -> EXIT -> DONE          (FAIL = hold in place)

  CRUISE    take off to fly_alt_m (5 m: the whole 1 m tag fits the Brio's view), fly straight ahead (yaw locked)
            looking for the QR with the sweep_jetson detector on every other frame. LANE: takeoff holds the pad and
            cruise steers back onto the straight line from the pad along the pad heading, so point the drone at the
            corridor before arming. The entrance is found by EITHER the green floor banner (down camera) OR the arch
            banner closer than arch_entry_m (front camera); then hold position on the lane level with the banner and
            descend to corridor_alt_m. Down camera: the green FLOOR banner, and a QR hint
            (a high-contrast patch, which works on a half-visible tag). On a hint: stop, centre on the patch and
            climb until the WHOLE tag is in view (a 1 m tag does not fit the Brio's ~0.95 m-tall view at 1.75 m),
            collect frames, file the majority-voted binary module grid (qr_out, .npy; never decoded) and descend
            again. Front camera: the green ARCH banner steers the approach laterally.
  ENTRY     floor banner confirmed = state change. Depth camera is only subscribed from here on. Slow down and
            line up with the corridor (depth walls; arch banner as lateral fallback).
  CORRIDOR  centre between the walls: lateral velocity from the centre-line offset, yaw from the wall direction.
            Forward speed is scaled down while the errors are large, so the drone never rushes a misaligned mouth.
  EXIT      both walls gone for end_hold_s -> fly exit_clear_m more (the walls end ~1 m ahead of the camera
            when they drop out of view), then
  DONE      hold position, keep publishing offboard setpoints, publish "DONE" on /corridor/status and write
            result_file. Offboard control stays with this node until the sweep node (sweep_integration_A.py) takes over:
            ros2 topic pub --once /search/start std_msgs/msg/String "{data: left}"

virtual_walls (default on; rods-only test): no depth camera. ENTRY centres on the arch banner; once centred, the entry pose
and heading are latched and imagined walls are held +-corridor_width_m/2 from that line for virtual_len_m, driven by
odometry; the corridor "ends" at virtual_len_m and EXIT/DONE run as usual.

Subscriptions are dropped as soon as a phase no longer needs them (down+front cameras after the corridor is
entered, depth after the exit).  Defaults are the real-flight setup (USB webcam JPEG + Orbbec Gemini, virtual walls).
Gazebo / real walls: -p use_sim_time:=true -p virtual_walls:=false -p down_compressed:=false and the sim image topics.
"""
import json
import math
import os
import time
from collections import deque

import cv2
import numpy as np
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy,
                       qos_profile_sensor_data)

from px4_msgs.msg import (OffboardControlMode, TrajectorySetpoint, VehicleAttitude, VehicleCommand,
                          VehicleLocalPosition, VehicleStatus)
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import String

import corridor_vision as cvz
from qr_signature import consensus as qr_consensus, signature as qr_signature


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def clamp(v, lim):
    return max(-lim, min(lim, v))


def slew(cur, tgt, rate, dt):
    return cur + clamp(tgt - cur, rate * dt)


def _plane(msg, itemsize, ch=1):
    a = np.frombuffer(msg.data, np.uint8 if itemsize == 1 else (np.float32 if itemsize == 4 else np.uint16))
    return a.reshape(msg.height, msg.step // itemsize)[:, :msg.width * ch]


def to_bgr(msg):
    if msg.encoding in ('rgb8', 'bgr8'):
        a = _plane(msg, 1, 3).reshape(msg.height, msg.width, 3)
        return np.ascontiguousarray(a[:, :, ::-1]) if msg.encoding == 'rgb8' else a
    return cv2.cvtColor(_plane(msg, 1), cv2.COLOR_GRAY2BGR)       # mono8


def to_depth_m(msg):
    if msg.encoding == '32FC1':
        return _plane(msg, 4)
    return _plane(msg, 2).astype(np.float32) / 1000.0              # 16UC1 millimetres (Orbbec driver)


class CorridorNav(Node):
    def __init__(self):
        super().__init__('corridor_nav')
        P = self.declare_parameter
        if not self.has_parameter('use_sim_time'):
            P('use_sim_time', False)     # real flight; pass -p use_sim_time:=true for Gazebo
        P('down_compressed', True)       # True: down_topic is a sensor_msgs/CompressedImage (JPEG), e.g. /camera/image_raw/compressed
        P('fly_alt_m', 5.0)              # takeoff + cruise + QR search height: metres above the PAD (see HEIGHT REFERENCE in _on_pos)
        P('corridor_alt_m', 1.75)        # drop to this once the corridor entrance is found
        P('green_hsv_lo', [35, 40, 25])  # HSV lower bound for the banners (H, S, V); lower S/V = darker greens pass
        P('green_hsv_hi', [98, 255, 255])  # upper bound; the real banner reads teal on the Brio (#5ca09f = H 90), sky starts ~H 100
        P('front_hfov', 1.5)             # Orbbec Gemini colour, for the arch-banner distance
        P('arch_entry_m', 6.0)           # arch banner closer than this = entrance found (no floor banner needed)
        P('height_settle_s', 5.0)        # don't arm until PX4's local height has stayed within height_settle_m for this long ...
        P('height_settle_m', 0.15)       # ... (a still-converging baro/EKF on the pad otherwise bakes its error into the flight)
        P('cruise_speed', 1.0)
        P('entry_speed', 0.5)
        P('corridor_speed', 1.0)
        P('exit_speed', 0.8)
        P('exit_clear_m', 1.8)
        P('end_hold_s', 0.5)
        P('min_corridor_m', 3.0)         # ignore "walls gone" until this far into the corridor
        P('corridor_width_m', 1.524)     # 5 ft
        P('arch_banner_w_m', 1.6)        # physical width of the banner on the arch
        P('down_hfov', 0.90)        # Brio 100
        P('depth_hfov', 1.57)
        P('down_topic', '/camera/image_raw/compressed')   # usb_camera_node.py -p encoding:=jpeg
        P('front_topic', '/camera/color/image_raw')       # Orbbec Gemini colour
        P('depth_topic', '/camera/depth/image_raw')       # Orbbec Gemini depth (16UC1, mm)
        P('qr_out', '~/qr_target.npy')   # bool module grid; sweep_movement_lanes_sig.py takes it as qr_target_image
        P('qr_max_alt_m', 6.0)           # highest the QR scan will climb (when only part of the tag is in view)
        P('qr_min_frames', 4)            # whole-tag frames that must agree before the grid is filed (sweep_jetson: SIG_MIN_FRAMES)
        P('tag_size_m', 1.0)             # printed QR side, for the expected size in pixels (ROI search, size gate)
        P('sig_grid', 29)                # module grid used when the size can't be estimated from the finder patterns
        P('sig_adaptive', False)         # adaptive instead of Otsu threshold for uneven outdoor light
        P('qr_scan_timeout_s', 20.0)
        P('qr_hint_std', 55.0)           # grey-level spread of a tile that counts as 'QR-like'
        P('virtual_walls', True)         # True: no depth walls needed (outdoor test with corner rods only); see _virtual_walls
        P('virtual_len_m', 6.0)          # corridor length to imagine once aligned at the entrance (> min_corridor_m)
        P('result_file', '~/corridor_result.json')
        P('debug_dir', '')               # '' = off; else CSV trace + snapshots are written here
        P('log_period_s', 1.0)           # status / camera / QR 'thinking' logs at most this often each; 0 = every frame
        g = lambda n: self.get_parameter(n).value
        self.p = {n: g(n) for n in (
            'fly_alt_m', 'corridor_alt_m', 'front_hfov', 'arch_entry_m', 'cruise_speed', 'entry_speed', 'corridor_speed', 'exit_speed', 'exit_clear_m',
            'end_hold_s', 'min_corridor_m', 'corridor_width_m', 'arch_banner_w_m',
            'down_hfov', 'depth_hfov', 'qr_max_alt_m', 'qr_min_frames', 'qr_scan_timeout_s', 'qr_hint_std', 'down_compressed',
            'virtual_walls', 'virtual_len_m', 'height_settle_s', 'height_settle_m',
            'tag_size_m', 'sig_grid', 'sig_adaptive', 'log_period_s')}
        self._log_t = {}
        cvz.GREEN_LO = np.array(g('green_hsv_lo'), np.uint8)
        cvz.GREEN_HI = np.array(g('green_hsv_hi'), np.uint8)
        self.v_origin = self.v_yaw = None
        self.qr_path = os.path.expanduser(g('qr_out'))
        self.result_path = os.path.expanduser(g('result_file'))
        self.debug_dir = os.path.expanduser(g('debug_dir'))
        if self.debug_dir:
            os.makedirs(self.debug_dir, exist_ok=True)
            self.trace = open(os.path.join(self.debug_dir, 'trace.csv'), 'w')
            self.trace.write('t,state,x,y,z,yaw_deg,vf,vr,ey,epsi_deg,walls,width,arch_ey,floor_hits\n')

        pub_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             history=HistoryPolicy.KEEP_LAST, depth=1)
        self.mode_pub = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', pub_qos)
        self.sp_pub = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', pub_qos)
        self.cmd_pub = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', pub_qos)
        self.status_pub = self.create_publisher(String, '/corridor/status', 10)

        self.ctl_group = MutuallyExclusiveCallbackGroup()
        self.vis_group = MutuallyExclusiveCallbackGroup()
        s = qos_profile_sensor_data
        self.create_subscription(VehicleLocalPosition, '/fmu/out/vehicle_local_position_v1', self._on_pos, s)
        self.create_subscription(VehicleAttitude, '/fmu/out/vehicle_attitude', self._on_att, s)
        self.create_subscription(VehicleStatus, f'/fmu/out/vehicle_status_v{VehicleStatus.MESSAGE_VERSION}', self._on_status, s)
        self.subs = {}                   # image subscriptions, created per phase
        self._sub('down', 'down_topic', self._on_down, CompressedImage if self.p['down_compressed'] else Image)
        self._sub('front', 'front_topic', self._on_front)

        self.pos = None
        self.yaw = None
        self.armed = False
        self.nav_state = None
        self.preflight_ok = False
        # HEIGHT REFERENCE (as in sweep_jetson_standalone.py): PX4's local z is relative to wherever the EKF
        # started, which can be metres off the pad. pad_h = PX4 local height of the pad, sampled once the
        # height has settled and before arming; every height here is local height - pad_h and every z
        # setpoint goes out as z_sp - pad_h. EKF height resets shift pad_h with them.
        self.pad_h = None
        self.h_hist = deque(maxlen=1000)  # (t, local height, z_valid) on the pad, for the pre-arm settle gate
        self._z_reset_ctr = None
        self.state = 'INIT'
        self.t_state = self.now()
        self.n = 0
        self.dt = 0.05
        self.z_sp = self.z_goal = -self.p['fly_alt_m']
        self.entrance_seen = False       # floor banner OR near arch banner
        # LANE: straight line from the pad along the pad heading (PX4 local frame, so GPS-fused when GPS is on).
        # Takeoff holds the pad, cruise steers back onto the line, the entrance stop is a point on it (entry_s m along).
        self.lane_o = self.lane_yaw = self.entry_s = None
        self.yaw_sp = None
        self.yaw_goal = None
        self.vf = self.vr = 0.0
        self.hold_xy = None
        self.state_pos = None
        # vision results (written by image callbacks, read by the control tick)
        self.qr_mode = 'idle'            # idle -> scan -> done | gave_up
        self.qr_frames = []              # whole-tag grids of the target
        self.qr_hint = None              # dict(t, cx, cy, W, H)
        self.qr_full_t = -1.0
        self.qr_last_uv = None           # pixel centre of the last whole-tag detection (ROI search seed)
        # QR detection as in sweep_jetson_standalone.py: aruco detector with the classic one as fallback, a CLAHE
        # retry, a high-resolution ROI search around where the tag was last seen and a full-frame search
        # downscaled to SEARCH_MAX_WIDTH; per-frame signatures fused by consensus, oldest half dropped on disagreement.
        self.detectors = ([cv2.QRCodeDetectorAruco()] if hasattr(cv2, 'QRCodeDetectorAruco') else []) + [cv2.QRCodeDetector()]
        self.clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8, 8))
        self.SEARCH_MAX_WIDTH = 2000
        self.ROI_MAX_PX = 1200
        self.SIZE_GATE = (0.4, 2.5)       # detected side / expected side
        self.SIG_MAX_FRAMES = 30
        self.qr_t0 = 0.0
        self.mono_warned = False
        self.arch = None                 # dict(t, ey, w, clipped, d)
        self.floor = None                # dict(t, ey): floor banner centre offset, right-positive
        self.floor_hits = 0
        self.down_sum = self.front_sum = 'no frame yet'   # one-phrase camera verdicts for the status line
        self.walls = None                # dict from corridor_from_depth + t
        self.walls_seen_t = -1.0
        self._walls_used_t = -1.0
        self.ey_f = self.epsi_f = None
        self.aligned_since = None
        self.no_walls_since = None
        self.front_n = self.down_n = 0
        self.dbg_saved = set()
        self.ctl_timer = self.create_timer(self.dt, self.tick, callback_group=self.ctl_group)
        self.get_logger().info('corridor_nav up: waiting for PX4 pose / attitude.')

    # ------------------------------------------------------------------ plumbing
    def now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def _every(self, key, period=None):
        """True at most once per period (log_period_s) per key: rate-limits the 'thinking' logs."""
        t, period = time.monotonic(), self.p['log_period_s'] if period is None else period
        if t - self._log_t.get(key, -1e9) < period:
            return False
        self._log_t[key] = t
        return True

    @staticmethod
    def _why_not(diag):
        """'no green' / why the biggest green blob was not taken as a banner."""
        if not diag['blobs']:
            return 'no green'
        rej = diag['rejected']
        return f'{rej[0][0]:.0%} green blob {rej[0][1]}' if rej else 'green specks only'

    def _sub(self, key, topic_param, cb, kind=Image):
        self.subs[key] = self.create_subscription(kind, self.get_parameter(topic_param).value, cb,
                                                  qos_profile_sensor_data, callback_group=self.vis_group)

    def _drop(self, *keys):
        for k in keys:
            if k in self.subs:
                self.destroy_subscription(self.subs.pop(k))
                self.get_logger().info(f'unsubscribed {k} camera (no longer needed)')

    def set_state(self, s):
        self.get_logger().info(f'STATE {self.state} -> {s}')
        self.state, self.t_state, self.state_pos = s, self.now(), None if self.pos is None else self.pos.copy()
        self.status_pub.publish(String(data=f'STATE:{s}'))

    def _on_pos(self, m):
        ctr = int(m.z_reset_counter)
        if self._z_reset_ctr is not None and ctr != self._z_reset_ctr:
            dz = float(m.delta_z)                    # new z = old z + delta_z
            if self.pad_h is None:
                self.h_hist.clear()                  # samples before the reset are in the old frame
            else:
                self.pad_h -= dz
                self.get_logger().warn(f'PX4 EKF height reset by {dz:+.2f} m -- pad zero shifted to keep the real height continuous.')
        self._z_reset_ctr = ctr
        self.pos = np.array([m.x, m.y, m.z])
        if self.pad_h is None:
            self.h_hist.append((self.now(), -float(m.z), bool(m.z_valid)))

    def _height_settled(self):
        """Pre-arm gate: PX4 preflight OK, z_valid, and local height steady over the last height_settle_s -> (ok, reason)."""
        win, tol = self.p['height_settle_s'], self.p['height_settle_m']
        if not self.preflight_ok:
            return False, 'PX4 preflight checks not passing'
        if not self.h_hist or not self.h_hist[-1][2]:
            return False, 'PX4 height not valid yet'
        now = self.h_hist[-1][0]
        if now - self.h_hist[0][0] < win:
            return False, f'collecting {win:.0f} s of height samples'
        hs = [h for t, h, _ in self.h_hist if now - t <= win]
        spread = max(hs) - min(hs)
        if spread > tol:
            return False, f'height moved {spread:.2f} m in the last {win:.0f} s (limit {tol:.2f} m)'
        return True, f'height steady within {spread:.2f} m over {win:.0f} s'

    def _on_att(self, m):
        w, x, y, z = m.q
        self.yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))

    def _on_status(self, m):
        self.armed = m.arming_state == VehicleStatus.ARMING_STATE_ARMED
        self.nav_state = m.nav_state
        self.preflight_ok = bool(m.pre_flight_checks_pass)

    def cmd(self, command, p1=0.0, p2=0.0):
        m = VehicleCommand()
        m.command, m.param1, m.param2 = command, float(p1), float(p2)
        m.target_system = m.target_component = m.source_system = m.source_component = 1
        m.from_external = True
        m.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.cmd_pub.publish(m)

    # ------------------------------------------------------------------ vision callbacks
    def _on_down(self, msg):
        self.down_n += 1                                # counted in every state: the status line shows the camera is alive
        if self.state not in ('CRUISE', 'ENTRY'):
            return
        if isinstance(msg, CompressedImage):
            bgr = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)    # JPEG is colour: green banner visible
            if bgr is None:
                self.get_logger().warn(f'DOWN: could not decode a {len(msg.data)} byte {msg.format} frame',
                                       throttle_duration_sec=2.0)
                return
            mono = False
        else:
            bgr = None
            mono = msg.encoding == 'mono8'
        if mono and not self.mono_warned:
            self.mono_warned = True
            self.get_logger().error('down camera is mono8: the green floor banner cannot be seen. '
                                    'Run usb_camera_node with -p encoding:=bgr8')
        if bgr is None and not mono:
            bgr = to_bgr(msg)
        gray = _plane(msg, 1) if mono else cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
        if self.state == 'CRUISE':
            if self.qr_mode == 'idle' and self.down_n % 2 == 0:
                h = cvz.qr_hint(gray, self.p['qr_hint_std'])
                if h:
                    self.qr_hint = dict(t=self.now(), cx=h[0], cy=h[1], W=gray.shape[1], H=gray.shape[0])
                    self.get_logger().debug(f'QR HINT: high-contrast patch at px ({h[0]:.0f},{h[1]:.0f}) '
                                            f'of {gray.shape[1]}x{gray.shape[0]}', throttle_duration_sec=1.0)
                else:                                       # sweep_jetson runs the full detector on every frame
                    self._scan_qr(gray)                     # a whole tag found here becomes the hint that starts the scan
            elif self.qr_mode == 'scan':
                h = cvz.qr_hint(gray, self.p['qr_hint_std'])
                if h:
                    self.qr_hint = dict(t=self.now(), cx=h[0], cy=h[1], W=gray.shape[1], H=gray.shape[0])
                self._scan_qr(gray)
        if bgr is None:
            return
        small = cv2.resize(bgr, (640, 360), interpolation=cv2.INTER_AREA)
        diag = {}
        b = cvz.green_blob(small, diag=diag)
        alt = -self.pos[2] - (self.pad_h or 0.0)
        view_w = 2 * max(alt, 0.5) * math.tan(self.p['down_hfov'] / 2)   # ground metres across
        if b:                                       # green_blob only returns long solid rectangles: any size is the banner
            self.floor_hits += 1
            fwd, ey, xy = self._down_to_ground(b['cx'] / b['W'], b['cy'] / b['H'], bgr.shape[0] / bgr.shape[1])
            self.floor = dict(t=self.now(), ey=ey, fwd=fwd, xy=xy, clipped_tb=b['clipped_tb'])
            if self.debug_dir and 'floor' not in self.dbg_saved and self.floor_hits >= 3:
                self.dbg_saved.add('floor')
                cv2.imwrite(os.path.join(self.debug_dir, 'floor_trigger.png'), small)
        else:
            self.floor_hits = 0
        if self._every('down'):
            self.down_sum = (f'BANNER {self.floor["fwd"]:+.1f}m ahead {self.floor["ey"]:+.1f}m rt ({self.floor_hits}/3)'
                             if b else self._why_not(diag))
            self.get_logger().debug(
                f'DOWN #{self.down_n} at {alt:.2f} m: green {diag["green_frac"]:.1%} of frame, {diag["blobs"]} blobs, '
                + (f'banner {b["w"] / b["W"] * view_w:.2f} m wide, aspect {b["aspect"]:.1f}, fill {b["fill"]:.2f}, '
                   f'clipped L/R {b["clipped_lr"]} T/B {b["clipped_tb"]}' if b else f'rejected: {diag["rejected"][:3]}'))
            if self.debug_dir and self._every('down_snap', 5.0):     # what the camera + mask looked like
                m = cv2.cvtColor(cvz.green_mask(small), cv2.COLOR_GRAY2BGR)
                cv2.imwrite(os.path.join(self.debug_dir, f'down_{self.down_n:06d}.jpg'), np.hstack([small, m]))

    def _detect_qr(self, gray, retry):
        variants = [gray] + ([self.clahe.apply(gray)] if retry else [])
        for img in variants:
            for det in self.detectors:
                try:
                    ok, pts = det.detectMulti(img)
                except cv2.error:
                    continue
                if ok and pts is not None and len(pts) > 0:
                    return [p.reshape(-1, 2).astype(np.float32) for p in pts]
        return []

    def _detect_in_roi(self, raw, uv, expected_px):
        H, W = raw.shape
        half = int(np.clip(3.0 * expected_px, 200, 1000))
        u, v = int(round(uv[0])), int(round(uv[1]))
        x0, x1 = max(0, u - half), min(W, u + half)
        y0, y1 = max(0, v - half), min(H, v + half)
        if x1 - x0 < 64 or y1 - y0 < 64:
            return []
        roi = np.ascontiguousarray(raw[y0:y1, x0:x1])
        s = min(1.0, self.ROI_MAX_PX / max(roi.shape))
        if s < 1.0:
            roi = cv2.resize(roi, None, fx=s, fy=s, interpolation=cv2.INTER_AREA)
        offs = np.array([x0, y0], dtype=np.float32)
        return [c / s + offs for c in self._detect_qr(roi, retry=True)]

    def _detect_full(self, raw):
        H, W = raw.shape
        s = min(1.0, self.SEARCH_MAX_WIDTH / W)
        img = cv2.resize(raw, None, fx=s, fy=s, interpolation=cv2.INTER_AREA) if s < 1.0 else raw
        return [c / s for c in self._detect_qr(img, retry=False)]

    def _pick_tag(self, cands, gray, expected_px, ref, edge_px=12):
        """Whole tag in frame (the grid needs all of it) and plausibly sized; closest to ref wins."""
        H, W = gray.shape
        lo, hi = self.SIZE_GATE
        best, best_d = None, 1e18
        for c in cands:
            if c.min() < edge_px or c[:, 0].max() > W - edge_px or c[:, 1].max() > H - edge_px:
                continue
            side = float(np.mean(np.linalg.norm(c - np.roll(c, -1, axis=0), axis=1)))
            if not lo <= side / expected_px <= hi:
                self.get_logger().debug(f'QR candidate dropped: size ratio {side / expected_px:.2f} outside {self.SIZE_GATE}',
                                        throttle_duration_sec=2.0)
                continue
            d = float(np.hypot(*(c.mean(axis=0) - ref)))
            if d < best_d:
                best, best_d = c, d
        return best

    def _scan_qr(self, gray):
        """Collect signatures of ONE whole tag (sweep_jetson pipeline); once enough agree, file their consensus."""
        H, W = gray.shape
        alt = -self.pos[2] - (self.pad_h or 0.0)
        f = (W / 2) / math.tan(self.p['down_hfov'] / 2)
        expected_px = f * self.p['tag_size_m'] / max(alt, 0.5)
        now = self.now()
        seed = self.qr_last_uv if now - self.qr_full_t < 1.0 else None
        hint = self.qr_hint
        if seed is None and hint and now - hint['t'] < 0.5:
            seed = (hint['cx'], hint['cy'])
        c = None
        if seed is not None:
            c = self._pick_tag(self._detect_in_roi(gray, seed, expected_px), gray, expected_px, np.array(seed))
        if c is None:
            c = self._pick_tag(self._detect_full(gray), gray, expected_px, np.array(seed or (W / 2, H / 2)))
        if self._every('qr_scan'):
            src = 'last tag' if seed is not None and seed == self.qr_last_uv else ('hint' if seed is not None else 'none')
            self.get_logger().debug(f'QR {self.qr_mode}: alt {alt:.2f} m, expect tag ~{expected_px:.0f} px, '
                                    f'seed {src}, whole tag {"FOUND" if c is not None else "not found"}, '
                                    f'{len(self.qr_frames)}/{self.p["qr_min_frames"]} frames collected')
        if c is None:
            return
        self.qr_last_uv, self.qr_full_t = tuple(c.mean(axis=0)), now
        self.qr_hint = dict(t=now, cx=float(self.qr_last_uv[0]), cy=float(self.qr_last_uv[1]), W=W, H=H)   # centre over it
        g, o = qr_signature(gray, c, grid=self.p['sig_grid'], adaptive=self.p['sig_adaptive'])
        self.qr_frames = (self.qr_frames + [(g, o)])[-self.SIG_MAX_FRAMES:]
        if len(self.qr_frames) < self.p['qr_min_frames']:
            return
        cons = qr_consensus(self.qr_frames)
        if cons is None:                                    # frames disagree (blur / mixed tags): wait for better ones
            self.qr_frames = self.qr_frames[len(self.qr_frames) // 2:]
            self.get_logger().warn('whole-tag frames disagree; collecting more', throttle_duration_sec=2.0)
            return
        if not cons[1]:                                     # sweep loads the .npy as oriented: need the finder layout
            self.get_logger().warn('tag orientation not resolved yet; collecting more', throttle_duration_sec=2.0)
            return
        np.save(self.qr_path, cons[0])
        cv2.imwrite(os.path.splitext(self.qr_path)[0] + '_preview.png', cvz.grid_preview(cons[0]))
        self.qr_mode = 'done'
        self.get_logger().info(f'QR target filed: {cons[0].shape[0]}x{cons[0].shape[0]} modules from '
                               f'{cons[2]} frames at {alt:.2f} m -> {self.qr_path}')
        # re-anchor the lane on the tag (same heading): the QR sits on the corridor line, the pad heading may not
        _, _, tag_xy = self._down_to_ground(self.qr_last_uv[0] / W, self.qr_last_uv[1] / H, H / W)
        d = tag_xy - self.lane_o
        shift = -d[0] * math.sin(self.lane_yaw) + d[1] * math.cos(self.lane_yaw)
        self.lane_o = tag_xy
        self.get_logger().info(f'LANE: re-anchored on the QR, {shift:+.2f} m right of the old line; straight on from here')

    def _on_front(self, msg):
        self.front_n += 1                               # counted in every state: the status line shows the camera is alive
        if self.state not in ('CRUISE', 'ENTRY'):
            return
        if self.front_n % 3:
            return
        diag = {}
        # no cy cut-off at 0.7 any more: from fly_alt_m the arch sits low in the frame
        b = cvz.green_blob(to_bgr(msg), min_frac=2e-4, max_cy_frac=0.95, diag=diag)
        # arch banner is ~5x wider than tall; a far-away floor strip is much flatter than that
        if b and b['w'] >= 8 and b['h'] >= 0.12 * b['w']:
            ey = cvz.banner_lateral_m(b, self.p['arch_banner_w_m'])
            f = (b['W'] / 2) / math.tan(self.p['front_hfov'] / 2)
            d = None if b['clipped_lr'] else f * self.p['arch_banner_w_m'] / b['w']   # pinhole range to the arch
            self.arch = dict(t=self.now(), ey=ey, w=b['w'], clipped=b['clipped_lr'], d=d)
            why = f'BANNER {d:.1f}m ahead {ey:+.2f}m rt' if d is not None else 'banner clipped at the side (no range)'
        elif b:
            why = f'{b["w"]}x{b["h"]}px blob too flat for the arch' if b['w'] >= 8 else 'blob too small'
        else:
            why = self._why_not(diag)
        if self._every('front'):
            self.front_sum = why
            self.get_logger().debug(f'FRONT #{self.front_n}: green {diag["green_frac"]:.1%} of frame, {diag["blobs"]} '
                                    f'blobs, {why}' + (f', aspect {b["aspect"]:.1f}, fill {b["fill"]:.2f}' if b else ''))

    def _on_depth(self, msg):
        r = cvz.corridor_from_depth(to_depth_m(msg), self.p['depth_hfov'], width_nom=self.p['corridor_width_m'])
        r['t'] = self.now()
        self.walls = r
        if r['walls']:
            self.walls_seen_t = r['t']

    # ------------------------------------------------------------------ control
    def fused_errors(self, now):
        """(ey, epsi) right-positive, low-passed; None when no fresh cue. Walls first, arch banner as lateral fallback."""
        w = self.walls
        if w and now - w['t'] < 0.4 and w['walls'] and w['t'] != self._walls_used_t:
            self._walls_used_t = w['t']
            a = 0.5
            self.ey_f = w['ey'] if self.ey_f is None else self.ey_f + a * (w['ey'] - self.ey_f)
            if w['epsi'] is not None:
                self.epsi_f = w['epsi'] if self.epsi_f is None else self.epsi_f + a * (w['epsi'] - self.epsi_f)
        if w and now - w['t'] < 0.4 and w['walls']:
            return self.ey_f, self.epsi_f
        ar = self.arch
        if self.state == 'ENTRY' and ar and now - ar['t'] < 0.5 and ar['ey'] is not None:
            return ar['ey'], None
        fl = self.floor                                   # either banner is enough to line up on
        if self.state == 'ENTRY' and fl and now - fl['t'] < 0.5:
            return fl['ey'], None
        return None, None

    def steer(self, ey, epsi):
        """Lateral velocity (right) and yaw-goal update from the current errors."""
        vr = clamp(0.8 * ey, 0.5) if ey is not None else 0.0
        if epsi is not None:
            self.yaw_goal = wrap(self.yaw + epsi)
        return vr

    def tick(self):
        now = self.now()
        self.n += 1
        mode = OffboardControlMode()
        mode.position = mode.velocity = True
        mode.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.mode_pub.publish(mode)
        if self.pos is None or self.yaw is None or now <= 0.0:
            if self._every('wait_px4', 2.0):
                self.get_logger().info(f'waiting for PX4: position {"ok" if self.pos is not None else "MISSING"}, '
                                       f'attitude {"ok" if self.yaw is not None else "MISSING"}, clock {now:.1f}')
            return
        if self.yaw_sp is None:
            self.yaw_sp = self.yaw_goal = self.yaw
        if self.state == 'TAKEOFF' and self.n % 20 == 0 and not (self.armed and self.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD):
            self.get_logger().debug(f'TAKEOFF: requesting OFFBOARD + ARM (armed={self.armed}, nav_state={self.nav_state})')
            self.cmd(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
            self.cmd(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)

        want_vf, want_vr, hold = 0.0, 0.0, None
        alt = -self.pos[2] - (self.pad_h or 0.0)         # height above the pad
        st = self.state
        if st == 'INIT':
            ok, why = self._height_settled()
            if self.n > 20 and ok:
                hs = [h for t, h, _ in self.h_hist if self.h_hist[-1][0] - t <= self.p['height_settle_s']]
                self.pad_h = float(np.median(hs))
                self.z_sp = 0.0                        # climb from the pad, slewed up to fly_alt_m
                self.lane_o, self.lane_yaw = self.pos[:2].copy(), self.yaw
                self.yaw_sp = self.yaw_goal = self.yaw  # fly the lane nose-first
                self.get_logger().info(f'Pad zero set ({why}): pad is {self.pad_h:+.2f} m in PX4 local height; '
                                       f'flying at {self.p["fly_alt_m"]:.2f} m, corridor at {self.p["corridor_alt_m"]:.2f} m above the pad.')
                self.get_logger().info(f'LANE: from pad N {self.lane_o[0]:.2f} E {self.lane_o[1]:.2f} along heading '
                                       f'{math.degrees(self.lane_yaw):.1f} deg (point the drone at the corridor before arming)')
                self.set_state('TAKEOFF')
        elif st == 'TAKEOFF':
            hold = self.lane_o                         # climb straight up over the pad, no drift before cruise
            if self.armed and alt > self.p['fly_alt_m'] - 0.15 and now - self.t_state > 3.0:
                self.set_state('CRUISE')
        elif st == 'CRUISE':
            hint = self.qr_hint
            if self.qr_mode == 'idle' and hint and now - hint['t'] < 0.4:
                self.qr_mode, self.qr_t0 = 'scan', now
                self.get_logger().info('QR-like pattern below: stopping, centring on it and climbing until the whole tag is in view')
            if self.qr_mode == 'scan':
                full_now = now - self.qr_full_t < 0.5
                self.z_goal = self.z_sp if full_now else -max(self.p['qr_max_alt_m'], self.p['fly_alt_m'])  # stop climbing once the tag fits
                if hint and now - hint['t'] < 0.5:                                    # steer over the patch (image up = forward)
                    f = (hint['W'] / 2) / math.tan(self.p['down_hfov'] / 2)
                    want_vf = clamp(0.8 * (hint['H'] / 2 - hint['cy']) / f * alt, 0.4)
                    want_vr = clamp(0.8 * (hint['cx'] - hint['W'] / 2) / f * alt, 0.3)
                if now - self.qr_t0 > self.p['qr_scan_timeout_s']:
                    self.qr_mode = 'gave_up'
                    self.get_logger().warn('QR scan timed out without a whole-tag grid -- continuing without a target')
            else:
                ar, fl = self.arch, self.floor
                arch_near = ar and now - ar['t'] < 0.6 and ar['d'] is not None and ar['d'] <= self.p['arch_entry_m']
                if not self.entrance_seen and (self.floor_hits >= 3 or arch_near):
                    self.entrance_seen = True
                    # stop ON the lane: level with the floor banner (it is usually still ~1 m ahead, at the top of the
                    # 5 m view) or right here for the arch. A plain 'speed 0' drifts / overshoots and the banner
                    # leaves the view as it shrinks on the way down.
                    self.entry_s = self._lane_s(fl['xy'] if self.floor_hits >= 3 else self.pos[:2])
                    self.get_logger().info('corridor entrance found (' + ('floor banner' if self.floor_hits >= 3 else
                                           f'arch banner {ar["d"]:.1f} m ahead') + f'): stopping {self.entry_s:.2f} m down '
                                           f'the lane ({self.entry_s - self._lane_s(self.pos[:2]):+.2f} m from here) and '
                                           f'dropping to {self.p["corridor_alt_m"]:.2f} m')
                    if self.qr_mode != 'done':
                        self.qr_mode = 'gave_up'
                        self.get_logger().warn('corridor entrance reached but no QR grid was filed -- continuing without a target')
                lane_err = -self._lane_right()                  # metres to steer right to get back on the lane
                if self.entrance_seen:
                    if fl and now - fl['t'] < 0.3 and not fl['clipped_tb']:   # whole banner height in view: refine the stop
                        self.entry_s = self._lane_s(fl['xy'])
                    self.z_goal = -self.p['corridor_alt_m']
                    want_vf = clamp(0.8 * (self.entry_s - self._lane_s(self.pos[:2])), 0.5)
                    want_vr = clamp(0.8 * lane_err, 0.5)
                    if abs(alt - self.p['corridor_alt_m']) < 0.25:
                        self._begin_entry()
                else:
                    self.z_goal = -self.p['fly_alt_m']
                    settled = abs(alt - self.p['fly_alt_m']) < 0.15
                    want_vf = min(self.p['cruise_speed'] if settled else 0.4, 0.5 * (now - self.t_state))   # gentle start
                    want_vr = clamp(0.8 * lane_err, 0.5)        # back onto the lane (also after a QR scan pulled it off)
                    # line up with the arch -- but hold the lane until the QR is filed or the arch is near, so the
                    # lane the tag sits in isn't abandoned early (arch banner 40 px wide ~ 9 m away)
                    if ar and now - ar['t'] < 0.6 and ar['ey'] is not None and ar['w'] >= (20 if self.qr_mode == 'done' else 40):
                        want_vr = clamp(0.5 * ar['ey'], 0.35)
        elif st in ('ENTRY', 'CORRIDOR'):
            if self.p['virtual_walls'] and st == 'CORRIDOR':
                self._virtual_walls(now)
            ey, epsi = self.fused_errors(now)
            want_vr = self.steer(ey, epsi)
            e = self.ey_f if (self.walls and now - self.walls['t'] < 0.4 and self.walls['walls']) else ey
            g = max(0.0, 1 - abs(e) / 0.35) * max(0.0, 1 - abs(self.epsi_f or 0.0) / 0.30) if e is not None else 0.0
            if st == 'ENTRY':
                want_vf = self.p['entry_speed'] * g
                if self.p['virtual_walls']:       # centred on the arch banner = at the entrance
                    ok = ey is not None and abs(ey) < 0.15
                else:
                    ok = (self.walls and self.walls['walls'] == 2 and now - self.walls['t'] < 0.4
                          and 0.9 < self.walls['width'] < 2.4 and abs(self.ey_f) < 0.15 and abs(self.epsi_f or 0) < 0.12)
                self.aligned_since = (self.aligned_since or now) if ok else None
                if self.aligned_since and now - self.aligned_since > 0.4:
                    self.v_origin, self.v_yaw = self.pos[:2].copy(), self.yaw
                    ar = self.arch                        # lined up from a distance: start the virtual corridor at the arch
                    if ar and now - ar['t'] < 0.5 and ar['d'] is not None:
                        self.v_origin += ar['d'] * np.array([math.cos(self.yaw), math.sin(self.yaw)])
                    self._drop('down', 'front')
                    self.set_state('CORRIDOR')
                elif now - self.t_state > 12.0:
                    self.fail('no corridor walls found within 12 s of the floor banner')
            else:
                fresh = self.walls and now - self.walls['t'] < 0.5 and self.walls['walls']
                want_vf = self.p['corridor_speed'] * max(g, 0.25) if fresh else 0.2
                dist = float(np.linalg.norm(self.pos[:2] - self.state_pos[:2]))
                gone = self.walls and now - self.walls['t'] < 0.5 and self.walls['walls'] == 0
                self.no_walls_since = (self.no_walls_since or now) if gone else None
                if self.no_walls_since and now - self.no_walls_since > self.p['end_hold_s'] \
                        and dist > self.p['min_corridor_m']:
                    self.get_logger().info(f'corridor end after {dist:.1f} m')
                    self.corridor_len = dist
                    self.set_state('EXIT')
                elif now - self.walls_seen_t > 2.5:
                    self.fail('lost the corridor walls inside the corridor')
        elif st == 'EXIT':
            want_vf = self.p['exit_speed']
            if np.linalg.norm(self.pos[:2] - self.state_pos[:2]) >= self.p['exit_clear_m']:
                self._drop('depth')
                self.hold_xy = self.pos[:2].copy()
                self.set_state('DONE')
                self._write_result()
        elif st in ('DONE', 'FAIL'):
            hold = self.hold_xy
            if st == 'DONE' and self.n % 20 == 0:
                self.status_pub.publish(String(data='DONE'))

        # yaw: slew the setpoint toward the goal (<= 25 deg/s); velocity: accel-limited
        self.z_sp = slew(self.z_sp, self.z_goal, 0.4, self.dt)
        self.yaw_sp = wrap(self.yaw_sp + clamp(wrap(self.yaw_goal - self.yaw_sp), math.radians(25) * self.dt))
        self.vf = slew(self.vf, want_vf, 2.5 if abs(want_vf) < abs(self.vf) else 1.0, self.dt)   # brake harder than accelerate
        self.vr = slew(self.vr, want_vr, 1.5, self.dt)
        self.publish_sp(hold)
        if self._every('status'):
            self._log_status(now, alt, want_vf, want_vr)
        if self.debug_dir and self.n % 2 == 0:
            self._trace(now)

    def _down_to_ground(self, nu, nv, aspect):
        """Down-camera point (nu, nv: 0..1 across / down the image; aspect = H / W) -> (metres ahead, metres right,
        NED xy on the ground). Image up = forward; flat ground at the current height above the pad."""
        view_w = 2 * max(-self.pos[2] - (self.pad_h or 0.0), 0.5) * math.tan(self.p['down_hfov'] / 2)
        ey, fwd = (nu - 0.5) * view_w, (0.5 - nv) * view_w * aspect
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        return fwd, ey, self.pos[:2] + np.array([fwd * c - ey * s, fwd * s + ey * c])

    def _lane_s(self, xy):
        """Metres along the lane from the pad to the point xy (NED north, east)."""
        d = xy - self.lane_o
        return d[0] * math.cos(self.lane_yaw) + d[1] * math.sin(self.lane_yaw)

    def _lane_right(self):
        """The drone's offset to the right of the lane line [m]."""
        d = self.pos[:2] - self.lane_o
        return -d[0] * math.sin(self.lane_yaw) + d[1] * math.cos(self.lane_yaw)

    def _log_status(self, now, alt, want_vf, want_vr):
        """The one periodic line: state, height, commanded speed, and only what the current state is waiting on."""
        st, ar, fl, w = self.state, self.arch, self.floor, self.walls
        if st in ('DONE', 'FAIL'):
            return
        f2 = lambda v: '-' if v is None else f'{v:+.2f}'
        s = f'{st} {now - self.t_state:.0f}s | alt {alt:.2f}/{-self.z_goal:.2f}'
        if st == 'INIT':
            s = f'{st} | waiting to arm: {self._height_settled()[1]}'
        elif st == 'TAKEOFF':
            s += f' | armed {"yes" if self.armed else "NO"}, offboard {"yes" if self.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD else "NO"}'
        else:
            s += f' | v fwd {want_vf:.1f} rt {want_vr:+.1f}'
        if st == 'CRUISE':
            s += f' | lane {self._lane_s(self.pos[:2]):.1f}m, {self._lane_right():+.2f} rt'
            if self.entrance_seen:
                s += f' | ENTRANCE: stop {self.entry_s - self._lane_s(self.pos[:2]):+.2f}m ahead'
            if self.qr_mode in ('idle', 'scan'):
                s += f' | QR {self.qr_mode}' + (f' {len(self.qr_frames)}/{self.p["qr_min_frames"]}' if self.qr_mode == 'scan' else '')
        if st in ('CRUISE', 'ENTRY'):
            s += f' | floor: {self.down_sum} | arch: {self.front_sum}'
        if st == 'ENTRY':
            ey, _ = self.fused_errors(now)
            s += (f' | offset {f2(ey)} (need <0.15), aligned '
                  f'{0.0 if not self.aligned_since else now - self.aligned_since:.1f}/0.4s')
        elif st == 'CORRIDOR':
            s += (f' | {np.linalg.norm(self.pos[:2] - self.state_pos[:2]):.1f}m in, walls {w and w["walls"]}, '
                  f'offset {f2(self.ey_f)}, heading {"-" if self.epsi_f is None else f"{math.degrees(self.epsi_f):+.0f}"}deg')
        elif st == 'EXIT':
            s += f' | {np.linalg.norm(self.pos[:2] - self.state_pos[:2]):.1f}/{self.p["exit_clear_m"]:.1f}m past the end'
        self.get_logger().info(s)

    def _virtual_walls(self, now):
        """Imagined straight corridor from the pose latched at the entrance: walls at +-corridor_width_m/2 along
        the entry heading for virtual_len_m. Fills self.walls like corridor_from_depth, from odometry instead."""
        d = self.pos[:2] - self.v_origin
        c, s = math.cos(self.v_yaw), math.sin(self.v_yaw)
        along = d[0] * c + d[1] * s
        right = -d[0] * s + d[1] * c                       # drone's offset to the right of the centre line
        inside = along < self.p['virtual_len_m']
        self.walls = dict(t=now, walls=2 if inside else 0, width=self.p['corridor_width_m'],
                          ey=-right, epsi=wrap(self.v_yaw - self.yaw))
        if inside:
            self.walls_seen_t = now

    def _begin_entry(self):
        if self.p['virtual_walls']:
            self.get_logger().info('CORRIDOR ENTRANCE -> corridor mode; VIRTUAL walls (depth camera not used)')
        else:
            self.get_logger().info('CORRIDOR ENTRANCE -> corridor mode; arming the depth camera')
            self.subs['depth'] = self.create_subscription(
                Image, self.get_parameter('depth_topic').value, self._on_depth, qos_profile_sensor_data,
                callback_group=self.vis_group)
        self.set_state('ENTRY')

    def fail(self, why):
        self.get_logger().error(f'FAIL: {why} -- holding position')
        self.hold_xy = self.pos[:2].copy()
        self.set_state('FAIL')
        self.status_pub.publish(String(data='FAIL:' + why))

    def publish_sp(self, hold):
        nan = float('nan')
        m = TrajectorySetpoint()
        # pad frame -> PX4 local z; before the pad zero is set (INIT, disarmed) just stream the current height
        z = float(self.z_sp) - self.pad_h if self.pad_h is not None else float(self.pos[2])
        if hold is None:
            c, s = math.cos(self.yaw), math.sin(self.yaw)
            m.position = [nan, nan, z]
            m.velocity = [self.vf * c - self.vr * s, self.vf * s + self.vr * c, nan]
        else:
            m.position = [float(hold[0]), float(hold[1]), z]
            m.velocity = [nan, nan, nan]
        m.yaw = float(self.yaw_sp)
        m.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.sp_pub.publish(m)

    def _write_result(self):
        res = dict(status='done', qr_target_grid=self.qr_path if self.qr_mode == 'done' else None,
                   exit_heading_deg=math.degrees(self.yaw), exit_pos_ned=self.pos.tolist(),
                   corridor_len_m=getattr(self, 'corridor_len', None), time=time.time())
        with open(self.result_path, 'w') as f:
            json.dump(res, f, indent=1)
        self.get_logger().info(f'DONE: {res}')

    def _trace(self, now):
        w = self.walls or {}
        f = lambda v: '' if v is None else f'{v:.3f}'
        self.trace.write(f"{now:.2f},{self.state},{self.pos[0]:.2f},{self.pos[1]:.2f},{self.pos[2]:.2f},"
                         f"{math.degrees(self.yaw):.1f},{self.vf:.2f},{self.vr:.2f},{f(self.ey_f)},"
                         f"{f(None if self.epsi_f is None else math.degrees(self.epsi_f))},{w.get('walls', '')},"
                         f"{f(w.get('width'))},{f(self.arch and self.arch['ey'])},{self.floor_hits}\n")
        self.trace.flush()


def main(args=None):
    rclpy.init(args=args)
    node = CorridorNav()
    ex = MultiThreadedExecutor(num_threads=3)
    ex.add_node(node)
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
