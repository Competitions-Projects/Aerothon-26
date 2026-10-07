#!/usr/bin/env python3
"""
PX4 offboard coverage-sweep + visual red-zone avoidance + target-QR search.

Mission: sweep the arena at 10 m, and whenever a QR tag is confirmed, descend
just far enough to decode it and compare against a target payload chosen from
a folder of reference codes. Non-matches are logged and skipped on later
passes; a match ends the mission at that spot. The red keep-out zone is NOT
given in advance -- it is a painted region the drone must recognise with the
downward camera (HSV colour threshold) and avoid, accumulating what it has
seen into a convex hull and replanning the remaining sweep around it whenever
that hull grows. A separate, always-on velocity/position safety filter keeps
every commanded motion outside the current keep-out estimate, independent of
whether replanning has caught up -- see the RED-ZONE SAFETY note below for
what this can and cannot guarantee.
"""
import glob
import json
import math
import os
import random
import threading
import time
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy, qos_profile_sensor_data
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
import cv2
import numpy as np

from sensor_msgs.msg import Image
from px4_msgs.msg import (
    OffboardControlMode, TrajectorySetpoint, VehicleCommand,
    VehicleCommandAck, VehicleLocalPosition, VehicleAttitude, VehicleStatus,
)

cv2.setNumThreads(3)   # Orin Nano: leave cores for the executor / PX4 link

try:
    from pyzbar.pyzbar import decode as _zbar_decode
except Exception:                       # noqa: BLE001
    _zbar_decode = None


# ======================================================================
# Geometry (pure numpy). All (north, east) in the drone's local frame [m].
# ======================================================================
def poly_sdist_many(P, poly):
    """Signed distance from points P (k,2) to polygon (m,2): <0 inside, >0 outside."""
    P = np.atleast_2d(np.asarray(P, float))
    poly = np.asarray(poly, float)
    A, B = poly, np.roll(poly, -1, axis=0)
    AB = B - A
    L2 = np.maximum((AB ** 2).sum(1), 1e-12)
    AP = P[:, None, :] - A[None, :, :]
    t = np.clip((AP * AB[None]).sum(2) / L2[None], 0.0, 1.0)
    proj = A[None] + t[..., None] * AB[None]
    dmin = np.linalg.norm(P[:, None, :] - proj, axis=2).min(1)
    x, y = P[:, 0][:, None], P[:, 1][:, None]
    x1, y1, x2, y2 = A[:, 0][None], A[:, 1][None], B[:, 0][None], B[:, 1][None]
    with np.errstate(divide='ignore', invalid='ignore'):
        xin = (x2 - x1) * (y - y1) / (y2 - y1) + x1
    inside = ((((y1 > y) != (y2 > y)) & (x < xin)).sum(1) % 2) == 1
    return np.where(inside, -dmin, dmin)


def convex_hull(points):
    """Andrew's monotone chain; returns CCW hull, no external deps."""
    pts = sorted(set((round(float(a), 4), round(float(b), 4)) for a, b in points))
    if len(pts) <= 2:
        return pts

    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

    lower = []
    for p in pts:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], p) <= 0:
            lower.pop()
        lower.append(p)
    upper = []
    for p in reversed(pts):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], p) <= 0:
            upper.pop()
        upper.append(p)
    return lower[:-1] + upper[:-1]


def poly_area(poly):
    poly = np.asarray(poly, float)
    x, y = poly[:, 0], poly[:, 1]
    return 0.5 * abs(np.dot(x, np.roll(y, -1)) - np.dot(y, np.roll(x, -1)))


def inflate_convex(poly, margin):
    poly = np.asarray(poly, float)
    n = len(poly)
    area2 = sum(poly[i][0] * poly[(i + 1) % n][1] - poly[(i + 1) % n][0] * poly[i][1] for i in range(n))
    ccw = area2 > 0
    out = []
    for i in range(n):
        p0, p1, p2 = poly[i - 1], poly[i], poly[(i + 1) % n]
        e1, e2 = p1 - p0, p2 - p1
        n1 = np.array([e1[1], -e1[0]]) if ccw else np.array([-e1[1], e1[0]])
        n2 = np.array([e2[1], -e2[0]]) if ccw else np.array([-e2[1], e2[0]])
        n1 /= (np.linalg.norm(n1) + 1e-12)
        n2 /= (np.linalg.norm(n2) + 1e-12)
        miter = (n1 + n2) / max(1.0 + float(n1 @ n2), 0.3)
        out.append(p1 + margin * miter)
    return np.array(out)


class ObstaclePlanner:
    """Shortest obstacle-avoiding paths (visibility graph over inflated convex obstacles)."""

    def __init__(self, obstacles_ne, margin, step=0.25):
        self.obs = [np.asarray(o, float) for o in obstacles_ne if len(o) >= 3]
        self.margin = float(margin)
        self.step = step
        self.nodes = []
        for o in self.obs:
            self.nodes += [tuple(p) for p in inflate_convex(o, self.margin + 0.3)]

    def points_free(self, P, floor=None):
        P = np.atleast_2d(P)
        ok = np.ones(len(P), bool)
        for k, o in enumerate(self.obs):
            m = self.margin if floor is None else min(self.margin, floor[k])
            ok &= poly_sdist_many(P, o) >= m - 1e-6
        return ok

    def segment_free(self, a, b):
        a, b = np.asarray(a, float), np.asarray(b, float)
        k = max(1, int(math.ceil(np.linalg.norm(b - a) / self.step)))
        s = np.linspace(0.0, 1.0, k + 1)[:, None]
        # A start already inside the margin (the reactive filter lets the drone
        # sit there) may leave it, but never get closer than where it began;
        # otherwise every hop from it is "blocked" and the route comes out empty.
        floor = [float(poly_sdist_many([a], o)[0]) for o in self.obs]
        return bool(self.points_free(a + (b - a) * s, floor).all())

    def path(self, a, b):
        a, b = np.asarray(a, float), np.asarray(b, float)
        if not self.obs or self.segment_free(a, b):
            return [tuple(b)]
        pts = [a, b] + [np.array(n) for n in self.nodes if self.points_free(np.array(n))[0]]
        V = len(pts)
        dist = [math.inf] * V
        prev = [-1] * V
        done = [False] * V
        dist[0] = 0.0
        for _ in range(V):
            u = min((i for i in range(V) if not done[i]), key=lambda i: dist[i], default=None)
            if u is None or math.isinf(dist[u]):
                break
            done[u] = True
            if u == 1:
                break
            for v in range(V):
                if done[v] or not self.segment_free(pts[u], pts[v]):
                    continue
                nd = dist[u] + float(np.linalg.norm(pts[u] - pts[v]))
                if nd < dist[v]:
                    dist[v], prev[v] = nd, u
        if prev[1] == -1:
            return None
        chain, i = [], 1
        while i != 0:
            chain.append(tuple(pts[i]))
            i = prev[i]
        return chain[::-1]


def _path_len(a, path):
    L, cur = 0.0, np.asarray(a, float)
    for p in path:
        L += float(np.linalg.norm(np.asarray(p) - cur))
        cur = np.asarray(p)
    return L


# ----------------------------------------------------------------------
# Bounds JSON loading (tolerant of arbitrary wrapper keys)
# ----------------------------------------------------------------------
def _nk(k):
    return ''.join(ch for ch in str(k).lower() if ch.isalnum())


def _is_num(v):
    return isinstance(v, (int, float)) and not isinstance(v, bool)


def _rect_from_dict(k):
    def pick(*names):
        for n in names:
            if _is_num(k.get(n)):
                return float(k[n])
        return None
    x0 = pick('xmin', 'nmin', 'north_min', 'minx')
    x1 = pick('xmax', 'nmax', 'north_max', 'maxx')
    y0 = pick('ymin', 'emin', 'east_min', 'miny')
    y1 = pick('ymax', 'emax', 'east_max', 'maxy')
    if None not in (x0, x1, y0, y1):
        return [(x0, y0), (x1, y0), (x1, y1), (x0, y1)]
    return None


def parse_bounds(obj, depth=0):
    """Find an area-bounds rectangle anywhere in the JSON (handles wrapper
    keys like "area_bounds": {...} without needing to name them)."""
    if isinstance(obj, dict):
        k = {_nk(a): b for a, b in obj.items()}
        r = _rect_from_dict(k)
        if r is not None:
            return r
        if depth < 3:
            for v in obj.values():
                r = parse_bounds(v, depth + 1)
                if r is not None:
                    return r
    return None


def load_area_bounds(path):
    """Returns bounds as [(north, east), ...] -- x_min/x_max/y_min/y_max map
    directly to local-frame north/east, since the arena is built around the
    drone's own start point (no ENU/world offset)."""
    with open(path) as f:
        data = json.load(f)
    bounds = parse_bounds(data)
    if bounds is None:
        keys = list(data.keys()) if isinstance(data, dict) else type(data).__name__
        raise ValueError(f"could not find x_min/x_max/y_min/y_max anywhere in the file (top-level: {keys})")
    return bounds


# ======================================================================
# Node
# ======================================================================
class PX4QRSweepNode(Node):
    def __init__(self):
        super().__init__('px4_qr_sweep_node')

        if not self.has_parameter('use_sim_time'):
            self.declare_parameter('use_sim_time', True)
        self.declare_parameter('bounds_file', '')
        self.declare_parameter('qr_target_folder', os.path.expanduser('~/qr_codes'))  # was '~/qr_targets' -- fixed to match the real folder
        self.declare_parameter('qr_target_file', '')     # exact filename to use; '' => pick randomly from the folder
        self.declare_parameter('results_file', '~/qr_results.json')
        self.declare_parameter('obstacle_margin', 0.6)   # required real clearance from the red zone -- close is fine, never inside
        self.declare_parameter('viz_file', '~/qr_mission_live.json')
        self.declare_parameter('sweep_speed', 2.5)        # m/s along sweep lanes: ~30% less time-to-target than 1.5 in the lane sim; decode was fine to 3.5 m/s in Gazebo
        self.declare_parameter('coverage_edge_m', 1.7)    # image-edge margin: ground this close to the footprint edge is NOT counted as seen.
                                                          # measured: ~0.6 decode margin + ~0.5 tilt + ~0.5 pose lag at 2.5 m/s (scale up for faster sweeps)
        self.declare_parameter('pose_lag_s', 0.18)        # camera frames lag the pose by ~0.16-0.22 s (measured in Gazebo): match each frame to the pose this much earlier
        self.declare_parameter('cam_aspect', 3496.0 / 4656.0)  # image height/width; width spans east, height spans north (yaw 0)

        qos_profile_pub = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.offboard_control_mode_publisher = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile_pub)
        self.trajectory_setpoint_publisher = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile_pub)
        self.vehicle_command_publisher = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos_profile_pub)

        self.vision_cb_group = MutuallyExclusiveCallbackGroup()
        self.control_cb_group = MutuallyExclusiveCallbackGroup()
        qos_image = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                               history=HistoryPolicy.KEEP_LAST, depth=1)
        self.image_sub = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, qos_image,
            callback_group=self.vision_cb_group)
        self.ack_sub = self.create_subscription(
            VehicleCommandAck, '/fmu/out/vehicle_command_ack', self.ack_callback, qos_profile_sensor_data)
        self.local_pos_sub = self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position_v1', self.local_pos_callback, qos_profile_sensor_data)
        self.attitude_sub = self.create_subscription(
            VehicleAttitude, '/fmu/out/vehicle_attitude', self.attitude_callback, qos_profile_sensor_data)
        # Tracks whether PX4 is still in OFFBOARD (it drops out silently on comms blips/failsafes).
        self.status_sub = self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status_v4', self.status_callback, qos_profile_sensor_data)

        # ---------------- Camera model (matches model.sdf) ----------------
        self.CAM_HFOV_RAD = 1.17
        self.R_cam2body = np.array([[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]])

        # ---------------- QR detection / decoding ----------------
        self.detectors = []
        if hasattr(cv2, 'QRCodeDetectorAruco'):
            self.detectors.append(('aruco', cv2.QRCodeDetectorAruco()))
        self.detectors.append(('classic', cv2.QRCodeDetector()))
        self.clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8, 8))
        self.get_logger().info(
            f"QR detectors: {[n for n, _ in self.detectors]}, zbar decoder: {_zbar_decode is not None}")

        self.TAG_SIZE_M = 0.5
        self.SEARCH_MAX_WIDTH = 2000
        self.ROI_MAX_PX = 1200
        self.SIZE_GATE = (0.4, 2.5)
        self.BLIND_ALT_AGL = 1.0

        # ---------------- Target (folder of 5 reference QR images) --------
        self.target_payload = None
        self.target_source = None
        self._load_target()

        # ---------------- Red-zone vision (HSV colour threshold) ----------
        # These thresholds assume a fairly saturated red paint/tape under
        # roughly neutral lighting -- tune against your actual arena; test
        # with a still image first (see the message accompanying this file).
        self.RED_HSV_LO1 = (0, 90, 60);   self.RED_HSV_HI1 = (10, 255, 255)
        self.RED_HSV_LO2 = (170, 90, 60); self.RED_HSV_HI2 = (180, 255, 255)
        self.RED_MIN_PIXEL_AREA = 400     # on the SEARCH_MAX_WIDTH-scaled frame
        self.RED_GROWTH_TRIGGER_M2 = 4.0  # replan once the estimated hull grows by this much
        self.REPLAN_COOLDOWN_S = 3.0      # ...and no more than once per this many seconds
        self.RED_OUTLIER_LINK_M = 6.0     # a new ground point must be this close to a trusted one to be accepted
        self.RED_BOOTSTRAP_PTS = 4         # ...except for the first few, which bootstrap the cluster
        self.TAG_EXCLUSION_RADIUS_M = 1.2  # red-zone points this close to a known/tracked tag are dropped
                                            # (almost certainly the tag's own material, not real zone paint)
        self.red_pts = []                 # accumulated ground points (list of (n,e))
        self.red_zone_hull = None         # convex hull of red_pts, or None until first detection
        self.red_zone_area = 0.0
        self._replanned_area = 0.0  # hull area as of the last actual replan
        self._need_replan = False
        self._last_replan_t = -1e9

        # ---------------- Target estimator (for whichever QR is in view) --
        self.CONFIRM_COUNT = 3
        self.GATE_BASE_M = 0.75
        self.GATE_PER_M = 0.10
        self.EMA_ALPHA = 0.35
        self.LOST_TIMEOUT_S = 4.0
        self.LAND_LOST_GRACE_S = 3.0   # tag unseen this long past LOST_TIMEOUT_S during PRECISION_LAND -> climb to re-acquire
        self.KNOWN_RADIUS_M = 1.2  # pre-filter only; payload-based dedup in _finish_visit is now the
                                   # authoritative check, so this just needs to roughly avoid wasted re-approaches
        self.PURSUIT_EXEMPT_RADIUS_M = 4.0  # generous vs KNOWN_RADIUS_M: this covers realistic ground-point
                                             # noise for the SAME tag at full sweep altitude, not tag-to-tag spacing

        self.lock = threading.Lock()
        self.target = None
        self.n_consistent = 0
        self.reject_streak = 0
        self.target_confirmed = False
        self.last_seen_t = -1e9
        self.vision_stage = "NONE"
        self.pos_hist = deque(maxlen=1000)
        self.att_hist = deque(maxlen=2000)

        self.known_tags = []
        self.known_positions = []   # historical positions for reporting/debugging
        # Do not use historical position as a hard QR-detection veto: the
        # projected ground point can move substantially as altitude/pose
        # changes. Payload-based dedup in _finish_visit() is authoritative.
        self.last_completed_ne = None
        self.last_completed_t = -1e9
        self.RECENT_TAG_SUPPRESS_RADIUS_M = 1.5
        self.RECENT_TAG_SUPPRESS_S = 8.0
        # Every ground estimate of every tag whose payload we've read. A
        # candidate near one of these is only accepted if it decodes, in-frame,
        # to a payload we haven't seen -- position says "maybe", payload decides.
        self.done_ne = []
        self.DONE_RADIUS_M = 3.0
        self.decoded = None
        self.want_decode = False
        self.results_file = os.path.expanduser(str(self.get_parameter('results_file').value))

        # ---------------- Sweep / mission parameters (tuned for speed) ----
        self.SWEEP_ALT = 10.0
        self.SWEEP_Z = -self.SWEEP_ALT
        self.SWEEP_SPEED = float(self.get_parameter('sweep_speed').value)
        self.COV_EDGE_M = float(self.get_parameter('coverage_edge_m').value)
        self.POSE_LAG_S = float(self.get_parameter('pose_lag_s').value)
        self.CAM_ASPECT = float(self.get_parameter('cam_aspect').value)
        self.COV_RES = 0.5            # coverage raster resolution [m]
        self.COV_MIN_CELLS = 3        # stop when fewer unseen free cells than this remain
        self.LANE_OVERSHOOT_M = 1.0   # run lanes this far past the footprint edge (turn cut-off, WP_TOL)
        self.LANE_SHIFTS_M = (0.0, -2.0, 2.0, -4.0, 4.0)   # sideways lane offsets tried so a lane can hug (but not enter) the zone's keep-out
        self.PLAN_TOP_K = 8           # candidates that get exact obstacle-aware routing per planning call
        self.WP_TOL = 0.8
        self.obstacle_margin = float(self.get_parameter('obstacle_margin').value)
        self.PLAN_BUFFER_M = 0.3          # extra planning margin for turn overshoot / tracking lag
        # Onset distance for the REACTIVE filter, kept independent of obstacle_margin
        # on purpose: shrinking the hard clearance (to let the drone read boundary
        # tags) must never also shrink how early it starts reacting, or momentum can
        # carry it across a margin that's now much thinner.
        self.SAFETY_BUFFER_M = 1.3
        # PRECISION_LAND specifically: the target was already vetted as
        # outside the polygon (the detection-candidate filter rejected
        # anything closer than -0.2 m sdist) before we ever committed to
        # landing on it, and the approach there is slow and deliberate --
        # the generous general-purpose margin/buffer above is for open
        # exploration near an evolving, uncertain zone, not this.
        self.PRECISION_MARGIN_M = 0.25
        self.PRECISION_BUFFER_M = 0.3

        self.READ_MIN_ALT = 2.0
        self.FINAL_ALT_AGL = 0.4   # touchdown trigger altitude for PRECISION_LAND -- was referenced but never defined
        self.LAND_CENTER_TOL = 0.15   # max lateral error to hand off to NAV_LAND
        self.LAND_CENTER_WAIT_S = 5.0
        self.LAND_BLIND_ALT_AGL = 2.0  # PRECISION_LAND only: below this, keep descending on the frozen target even if the
                                       # tag is lost (it drops out of view/focus at ~1.5 m); previously climbed back to re-acquire
        self.READ_DESCENT_RATE = 0.5
        self.READ_FLOOR_DWELL_S = 3.0
        self.CLIMB_RATE = 1.0
        self.APPROACH_TOL = 0.4
        self.APPROACH_TICKS = 6
        self.APPROACH_TIMEOUT_S = 30.0
        self.READ_ABORT_S = 8.0
        self.ALIGN_KP = 1.0
        self.MAX_HORIZ_SPEED = 1.5

        self.current_pos = [0.0, 0.0, 0.0]
        self.vehicle_q = [1.0, 0.0, 0.0, 0.0]
        self.offboard_counter = 0
        self.vehicle_nav_state = None
        self.vehicle_armed = False
        self._last_offboard_reclaim_t = -1e9
        self.takeoff_ticks = 0
        self.state = "INIT"
        self.route = deque()               # deque of (lane_idx, (n,e)); lane_idx -1 = transit-only
        self.saved_route = []
        self.resume_pos = (0.0, 0.0)
        self.hold_ne = (0.0, 0.0)
        self.pursuit_ne = None     # fixed world position of the tag currently being approached/landed on
        self.landing_target_ne = None  # frozen target position used during PRECISION_LAND
        self.land_lost_t0 = None       # when the tag was first judged lost during PRECISION_LAND
        self.approach_last_tgt = None   # last good tgt during APPROACH, so a tracking gap coasts instead of aborting
                                    # so vision keeps refining its position through PRECISION_LAND
        self.visit_t0 = 0.0
        self.floor_t0 = None
        self._centered_ticks = 0
        self.approach_route = deque()
        self._approach_replan_t = -1e9
        self.APPROACH_REPLAN_S = 2.0      # refresh the routed path this often as the target estimate refines
        self._progress_best = None        # stuck/no-progress watchdog, shared by APPROACH/READ
        self._progress_t = 0.0
        self.STUCK_TIMEOUT_S = 8.0         # abandon the visit if distance-to-target hasn't improved in this long
        self.PROGRESS_EPS_M = 0.3          # ...must improve by at least this much to count as progress (ignores noise)
        self.z_cmd = self.SWEEP_Z
        self.CONTROL_DT = 0.05
        self.area_ok = False
        self.planner = ObstaclePlanner([], self.obstacle_margin + self.PLAN_BUFFER_M)
        self.bounds_ne = None
        self.seen = None                # coverage raster: True where the camera has reliably seen the ground
        self.dead_segs = set()          # lane segments that were flown (or abandoned) without revealing anything new
        self._cur_seg = None
        self._seg_before = 0
        self._hull_mask = None
        self._hull_mask_key = None
        self.viz_cells = []
        self._sweep_stuck_best = None   # stuck-progress watchdog for route-following (mirrors APPROACH's)
        self._sweep_stuck_t = 0.0

        # ---- Live 2D viz snapshot (read by qr_mission_viewer.py) ----
        # Purely observational: dumps existing state to a file, no effect on flight logic.
        self.viz_file = os.path.expanduser(str(self.get_parameter('viz_file').value))
        self.trail = deque(maxlen=3000)
        self._viz_tick = 0
        self._trail_tick = 0

        self._load_area()

        self.timer = self.create_timer(self.CONTROL_DT, self.timer_callback,
                                       callback_group=self.control_cb_group)

    # ------------------------------------------------------------------
    # Setup
    # ------------------------------------------------------------------
    def _load_area(self):
        given = str(self.get_parameter('bounds_file').value or '')
        home = os.path.expanduser('~')
        if given:
            paths = [os.path.expanduser(given)]
        else:
            paths = [p for p in sorted(glob.glob(os.path.join(home, '*.json')))
                     if os.path.abspath(p) != os.path.abspath(self.results_file)]
        errors = []
        for p in paths:
            try:
                self.bounds_ne = load_area_bounds(p)
            except Exception as e:                       # noqa: BLE001
                errors.append(f"{p}: {e}")
                continue
            self.area_ok = True
            b = np.array(self.bounds_ne)
            self.get_logger().info(
                f"Loaded area from {p}: north [{b[:, 0].min():.1f}, {b[:, 0].max():.1f}], "
                f"east [{b[:, 1].min():.1f}, {b[:, 1].max():.1f}] (local frame, x=north/y=east)")
            return
        self.get_logger().error(
            "No usable bounds JSON found. Set -p bounds_file:=/path/file.json. Tried: "
            + "; ".join(errors or ['(no ~/*.json files)']))

    def _load_target(self):
        folder = os.path.expanduser(str(self.get_parameter('qr_target_folder').value))
        chosen_name = str(self.get_parameter('qr_target_file').value)
        if not os.path.isdir(folder):
            self.get_logger().warn(f"No target folder at {folder}; running in read-all mode.")
            return
        files = sorted(sum((glob.glob(os.path.join(folder, ext))
                            for ext in ('*.png', '*.jpg', '*.jpeg', '*.bmp')), []))
        if not files:
            self.get_logger().warn(f"No image files in {folder}; running in read-all mode.")
            return
        if chosen_name:
            matches = [f for f in files if os.path.basename(f) == chosen_name]
            pick = matches[0] if matches else random.choice(files)
        else:
            pick = random.choice(files)
        img = cv2.imread(pick, cv2.IMREAD_GRAYSCALE)
        if img is None:
            self.get_logger().warn(f"Could not read {pick}; running in read-all mode.")
            return
        text = None
        for _, det in self.detectors:
            try:
                txt, _, _ = det.detectAndDecode(img)
            except cv2.error:
                txt = ''
            if txt:
                text = txt
                break
        if text is None and _zbar_decode is not None:
            res = _zbar_decode(img)
            if res:
                text = res[0].data.decode('utf-8', 'replace')
        if text is None:
            self.get_logger().warn(f"Could not decode a QR from {pick}; running in read-all mode.")
            return
        self.target_payload, self.target_source = text, pick
        self.get_logger().info(f"Mission target: '{text}' (chosen from {os.path.basename(pick)})")

    # ------------------------------------------------------------------
    # Coverage planning
    # ------------------------------------------------------------------
    def _start_sweep(self, pos):
        # Anchor the bounds to wherever the drone ACTUALLY is right now,
        # not to whatever PX4 happens to call local (0,0). PX4/Gazebo SITL
        # typically sets the EKF's local-position origin independently of
        # a model's spawn pose (PX4_GZ_MODEL_POSE moves the model, not
        # necessarily the local-frame origin), so local (0,0) does not
        # reliably coincide with where the vehicle actually starts. Shifting
        # the whole bounds rectangle so the corner nearest (0,0) in the JSON
        # lands on the drone's real starting position keeps the JSON's
        # intended layout -- dimensions, and which corner the drone starts
        # in -- correct regardless of that offset. The red zone needs no
        # such shift: it's detected live in the vehicle's own local frame,
        # so it's already correct no matter where that frame's origin is.
        bounds = np.asarray(self.bounds_ne, float)
        nearest_corner = min(bounds.tolist(), key=lambda c: math.hypot(c[0], c[1]))
        shift = np.array([pos[0], pos[1]]) - np.array(nearest_corner)
        if np.hypot(*shift) > 0.5:
            self.bounds_ne = [tuple(np.array(p) + shift) for p in self.bounds_ne]
            self.get_logger().info(
                f"Anchoring area bounds to the drone's actual start position "
                f"(shift: n={shift[0]:+.1f}, e={shift[1]:+.1f})")

        self._init_coverage()
        self._rebuild_planner()
        self.route = deque()
        hw_e, hh_e = self._reliable_half_sizes()
        self.get_logger().info(
            f"Lane sweep: {len(self._lane_es())} lanes {2 * hw_e:.1f} m apart along north, reliable footprint "
            f"{2 * hw_e:.1f} x {2 * hh_e:.1f} m (edge margin {self.COV_EDGE_M:.1f} m), "
            f"{self.sweep_speed_str()} m/s, {self.seen.size} coverage cells")
        if not self._plan_next_segment(pos):
            self.get_logger().error("Coverage planning produced no lanes. Landing.")
            self._finish_mission(pos)
            return
        self.state = "SWEEP"

    def sweep_speed_str(self):
        return f"{self.SWEEP_SPEED:.1f}"

    def _rebuild_planner(self):
        # self.planner is shared by every consumer of obstacle-aware pathing
        # (SWEEP's route, APPROACH's routed pursuit, RETURN's path home) --
        # it MUST be rebuilt from the live-detected zone whenever that
        # estimate changes, or every planner.path() call silently falls
        # back to a naive straight line that ignores the obstacle entirely,
        # leaving the reactive safety filter to catch 100% of the avoidance
        # burden on its own -- which it can only do by sliding along an
        # edge, never by routing around one.
        self.planner = ObstaclePlanner([o.tolist() for o in self._current_obstacles()],
                                       self.obstacle_margin + self.PLAN_BUFFER_M)

    # ------------------------------------------------------------------
    # Coverage raster + lane planner
    # ------------------------------------------------------------------
    def _init_coverage(self):
        b = np.asarray(self.bounds_ne, float)
        self.cov_n0, self.cov_e0 = float(b[:, 0].min()), float(b[:, 1].min())
        self.span_n = float(b[:, 0].max()) - self.cov_n0
        self.span_e = float(b[:, 1].max()) - self.cov_e0
        r = self.COV_RES
        nr, nc = max(1, int(math.ceil(self.span_n / r))), max(1, int(math.ceil(self.span_e / r)))
        self.seen = np.zeros((nr, nc), bool)
        cn = self.cov_n0 + (np.arange(nr) + 0.5) * r
        ce = self.cov_e0 + (np.arange(nc) + 0.5) * r
        self._cov_centres = np.stack(np.meshgrid(cn, ce, indexing='ij'), -1).reshape(-1, 2)
        self.dead_segs, self._cur_seg, self._hull_mask, self._hull_mask_key = set(), None, None, None
        blk = 5                                          # 2.5 m cells for the live viewer
        self._viz_blk = blk
        self._viz_nb = (int(math.ceil(nr / blk)), int(math.ceil(nc / blk)))
        self.viz_cells = [[round(self.cov_n0 + (i + 0.5) * blk * r, 2), round(self.cov_e0 + (j + 0.5) * blk * r, 2)]
                          for i in range(self._viz_nb[0]) for j in range(self._viz_nb[1])]

    def _reliable_half_sizes(self, height=None):
        """Half-width (east) / half-height (north) of the part of the image trusted for detection."""
        hw = (self.SWEEP_ALT if height is None else height) * math.tan(self.CAM_HFOV_RAD / 2.0)
        return max(hw - self.COV_EDGE_M, 0.5), max(hw * self.CAM_ASPECT - self.COV_EDGE_M, 0.5)

    def _lane_es(self):
        hw_e, _ = self._reliable_half_sizes()
        nl = max(1, int(math.ceil(self.span_e / (2.0 * hw_e))))
        pitch = self.span_e / nl
        return [self.cov_e0 + pitch * (k + 0.5) for k in range(nl)]

    def _cov_slices(self, n, e, hh, hw):
        r = self.COV_RES
        i0 = max(0, int(math.floor((n - hh - self.cov_n0) / r))); i1 = min(self.seen.shape[0], int(math.ceil((n + hh - self.cov_n0) / r)))
        j0 = max(0, int(math.floor((e - hw - self.cov_e0) / r))); j1 = min(self.seen.shape[1], int(math.ceil((e + hw - self.cov_e0) / r)))
        return i0, i1, j0, j1

    def _mark_seen(self, pos):
        h = -pos[2]
        if self.seen is None or h < 0.6 * self.SWEEP_ALT:
            return
        hw_e, hh_e = self._reliable_half_sizes(h)
        i0, i1, j0, j1 = self._cov_slices(pos[0], pos[1], hh_e, hw_e)
        if i1 > i0 and j1 > j0:
            self.seen[i0:i1, j0:j1] = True

    def _unseen_free(self):
        """Ground not yet seen and not already known to be red zone (inside the hull)."""
        m = ~self.seen
        hull = self.red_zone_hull
        if hull is not None and len(hull) >= 3:
            key = (len(hull), round(self.red_zone_area, 2))
            if key != self._hull_mask_key:
                self._hull_mask = (poly_sdist_many(self._cov_centres, np.asarray(hull, float)) < 0).reshape(self.seen.shape)
                self._hull_mask_key = key
            m = m & ~self._hull_mask
        return m

    def _lane_clearance(self):
        """Keep lane waypoints outside the reactive filter's onset (margin + buffer) of the zone,
        otherwise the filter stalls the drone short of them."""
        return self.obstacle_margin + self.SAFETY_BUFFER_M + 0.2

    def _clear_of_zone(self, P):
        P = np.atleast_2d(P)
        ok = np.ones(len(P), bool)
        for obs in self._current_obstacles():
            ok &= poly_sdist_many(P, obs) >= self._lane_clearance()
        return ok

    def _segment_candidates(self, m):
        """(key, p1, p2, gain): lane intervals clipped to zone-clear ground, trimmed to the rows that
        still hold unseen ground. gain = unseen cells under the swept footprint (summed-area table)."""
        hw_e, hh_e = self._reliable_half_sizes()
        r = self.COV_RES
        nr, nc = m.shape
        sat = np.zeros((nr + 1, nc + 1)); sat[1:, 1:] = m.cumsum(0).cumsum(1)
        e_lo, e_hi = self.cov_e0 + 0.5, self.cov_e0 + self.span_e - 0.5
        ns = np.arange(self.cov_n0, self.cov_n0 + self.span_n + 1e-9, 0.25)
        cands = []
        for k, e0 in enumerate(self._lane_es()):
            for dl in self.LANE_SHIFTS_M:
                e = min(max(e0 + dl, e_lo), e_hi)
                j0 = max(0, int(math.floor((e - hw_e - self.cov_e0) / r))); j1 = min(nc, int(math.ceil((e + hw_e - self.cov_e0) / r)))
                rows = np.where(m[:, j0:j1].any(1))[0]
                if len(rows) == 0:
                    continue
                row_n = self.cov_n0 + (rows + 0.5) * r
                free = self._clear_of_zone(np.stack([ns, np.full_like(ns, e)], 1))
                runs, st = [], None
                for i, f in enumerate(free):
                    if f and st is None:
                        st = i
                    if (not f or i == len(free) - 1) and st is not None:
                        runs.append((ns[st], ns[i if f else i - 1])); st = None
                for a, b in runs:
                    sel = (row_n >= a - hh_e) & (row_n <= b + hh_e)
                    if not sel.any():
                        continue
                    rmin = self.cov_n0 + rows[sel].min() * r
                    rmax = self.cov_n0 + (rows[sel].max() + 1) * r
                    n1 = min(max(a, rmin + hh_e - self.LANE_OVERSHOOT_M), b)
                    n2 = max(min(b, rmax - hh_e + self.LANE_OVERSHOOT_M), a)
                    if n1 > n2:
                        n1 = n2 = min(max((rmin + rmax) / 2.0, a), b)
                    key = (k, round(float(e), 1), round(float(a), 1), round(float(b), 1), round(float(n1), 1), round(float(n2), 1))
                    if key in self.dead_segs:
                        continue
                    i0, i1, jj0, jj1 = self._cov_slices((n1 + n2) / 2.0, e, (n2 - n1) / 2.0 + hh_e, hw_e)
                    gain = float(sat[i1, jj1] - sat[i0, jj1] - sat[i1, jj0] + sat[i0, jj0])
                    if gain >= self.COV_MIN_CELLS:
                        cands.append((key, (float(n1), e), (float(n2), e), gain))
        return cands

    def _plan_next_segment(self, pos):
        """Pick the lane segment with the most new ground per second (approach included),
        route to it around the zone, and queue it. False when nothing left to cover."""
        m = self._unseen_free()
        if int(m.sum()) < self.COV_MIN_CELLS:
            return False
        here = np.array([pos[0], pos[1]])
        spd = self.SWEEP_SPEED

        def rough(c):                                   # straight-line estimate, to pick who gets exact routing
            _, p1, p2, gain = c
            near = min(np.linalg.norm(here - p1), np.linalg.norm(here - p2))
            return gain / (near / spd + np.linalg.norm(np.array(p1) - np.array(p2)) / spd + 2.0)
        best, best_val = None, -1.0
        for key, p1, p2, gain in sorted(self._segment_candidates(m), key=rough, reverse=True)[:self.PLAN_TOP_K]:
            for first, second in ((p1, p2), (p2, p1)):
                path = self.planner.path(here, first)
                if path is None:
                    continue
                full = list(path) + [second]
                val = gain / (_path_len(here, full) / spd + len(full))
                if val > best_val:
                    best, best_val = (key, path, second), val
        if best is None:
            return False
        key, path, second = best
        k = key[0]
        self.route = deque([(-1, tuple(p)) for p in path[:-1]] + [(k, tuple(path[-1])), (k, tuple(second))])
        self._cur_seg, self._seg_before = key, int(m.sum())
        return True

    def _replan_sweep(self, pos):
        """Re-route the goals still queued around the (grown) zone; drop the segment if a goal
        is now unreachable, then pick whatever is best next. Coverage state is never lost."""
        self._rebuild_planner()
        goals = [(idx, pt) for idx, pt in self.route if idx >= 0]
        route, cur, ok = [], np.array([pos[0], pos[1]]), bool(goals)
        for idx, pt in goals:
            path = self.planner.path(cur, pt) if bool(self._clear_of_zone(np.array(pt))[0]) else None
            if path is None:
                ok = False
                break
            route += [(-1, tuple(x)) for x in path[:-1]] + [(idx, tuple(path[-1]))]
            cur = np.asarray(pt, float)
        if ok:
            self.route = deque(route)
        else:
            if goals and self._cur_seg is not None:
                self.dead_segs.add(self._cur_seg)
            self.route, self._cur_seg = deque(), None
            self._plan_next_segment(pos)
        self.get_logger().info(
            f"Replanned around detected red zone: {int(self._unseen_free().sum())} unseen cell(s) left, "
            f"{len(self.dead_segs)} segment(s) written off")

    def _viz_cell_status(self):
        done = self.seen if self._hull_mask is None else (self.seen | self._hull_mask)
        blk = self._viz_blk
        visited, blocked = [], []
        for i in range(self._viz_nb[0]):
            for j in range(self._viz_nb[1]):
                sl = (slice(i * blk, (i + 1) * blk), slice(j * blk, (j + 1) * blk))
                idx = i * self._viz_nb[1] + j
                if self._hull_mask is not None and self._hull_mask[sl].all():
                    blocked.append(idx)
                elif done[sl].all():
                    visited.append(idx)
        return visited, blocked

    def _follow_route(self, pos, plan_more=True):
        p = np.array([pos[0], pos[1]])
        while self.route and np.linalg.norm(np.array(self.route[0][1]) - p) < self.WP_TOL:
            self.route.popleft()
            self._sweep_stuck_best = None   # fresh waypoint, fresh progress baseline
        if not self.route:
            if plan_more and self._cur_seg is not None and int(self._unseen_free().sum()) >= self._seg_before:
                self.dead_segs.add(self._cur_seg)   # flew it, saw nothing new: don't pick it again
            self._cur_seg = None
            if not (plan_more and self._plan_next_segment(pos)):
                self.publish_setpoint_safe(pos, x=pos[0], y=pos[1], z=self.SWEEP_Z)
                return False
        d = np.array(self.route[0][1]) - p
        dist = float(np.linalg.norm(d))

        # The zone keeps growing between replans, so a waypoint that was safe when planned can
        # end up too close; the reactive filter alone then just slides along the edge. Write the
        # segment off and replan once progress has clearly stalled.
        now = self._now_s()
        if self._sweep_stuck_best is None or dist < self._sweep_stuck_best - self.PROGRESS_EPS_M:
            self._sweep_stuck_best, self._sweep_stuck_t = dist, now
        elif now - self._sweep_stuck_t > self.STUCK_TIMEOUT_S:
            self.get_logger().warn(
                f"No progress toward next sweep waypoint for {self.STUCK_TIMEOUT_S:.0f}s "
                f"(stuck {dist:.1f}m away) -- writing off the segment and replanning.")
            self._sweep_stuck_best = None
            self._need_replan = False
            self._last_replan_t = now
            self._replanned_area = self.red_zone_area
            if self._cur_seg is not None:
                self.dead_segs.add(self._cur_seg)
            self.route, self._cur_seg = deque(), None
            self._replan_sweep(pos)
            return True

        speed = min(self.SWEEP_SPEED, max(0.4, 0.8 * dist))
        v = d / max(dist, 1e-6) * speed
        self.publish_setpoint_safe(pos, z=self.SWEEP_Z, vx=float(v[0]), vy=float(v[1]))
        return True

    # ------------------------------------------------------------------
    # Math helpers
    # ------------------------------------------------------------------
    @staticmethod
    def _quat_to_rotmat(q):
        w, x, y, z = q
        return np.array([
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w),     2 * (x * z + y * w)],
            [2 * (x * y + z * w),     1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w),     2 * (y * z + x * w),     1 - 2 * (x * x + y * y)],
        ])

    @staticmethod
    def _clamp(val, limit):
        return max(-limit, min(limit, val))

    def _now_s(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _project_world_to_pixel(self, tgt_ne, pos, q, W, H, f):
        d_world = np.array([tgt_ne[0] - pos[0], tgt_ne[1] - pos[1], -pos[2]])
        d_cam = self.R_cam2body.T @ (self._quat_to_rotmat(q).T @ d_world)
        if d_cam[2] <= 1e-3:
            return None
        return (W / 2.0 + f * d_cam[0] / d_cam[2], H / 2.0 + f * d_cam[1] / d_cam[2])

    def _ground_point(self, uv, pos, q, W, H, f):
        ray_cam = np.array([(uv[0] - W / 2.0) / f, (uv[1] - H / 2.0) / f, 1.0])
        rw = self._quat_to_rotmat(q) @ (self.R_cam2body @ ray_cam)
        if rw[2] < 0.1:
            return None
        t = -pos[2] / rw[2]
        return (pos[0] + t * rw[0], pos[1] + t * rw[1])

    # ------------------------------------------------------------------
    # Telemetry
    # ------------------------------------------------------------------
    def local_pos_callback(self, msg):
        p = [float(msg.x), float(msg.y), float(msg.z)]
        self.current_pos = p
        with self.lock:
            self.pos_hist.append((self._now_s(), p[0], p[1], p[2]))

    def attitude_callback(self, msg):
        q = [float(msg.q[0]), float(msg.q[1]), float(msg.q[2]), float(msg.q[3])]
        self.vehicle_q = q
        with self.lock:
            self.att_hist.append((self._now_s(), q))

    def ack_callback(self, msg):
        self.get_logger().info(f"[PX4 ACK] Cmd: {msg.command} -> Result: {msg.result}")

    def status_callback(self, msg):
        self.vehicle_nav_state = msg.nav_state
        self.vehicle_armed = (msg.arming_state == VehicleStatus.ARMING_STATE_ARMED)

    def _pose_at(self, t):
        with self.lock:
            pos_s, att_s = list(self.pos_hist), list(self.att_hist)
        if not pos_s or not att_s:
            return None
        p = min(pos_s, key=lambda s: abs(s[0] - t))
        a = min(att_s, key=lambda s: abs(s[0] - t))
        return np.array(p[1:4]), a[1]

    # ------------------------------------------------------------------
    # RED-ZONE SAFETY
    #
    # `_current_obstacles()` returns the best current estimate (a convex
    # hull of every ground point the camera has confirmed as red so far).
    # `_safe_velocity` / `_safe_position` are applied to EVERY horizontal
    # command in the whole node (see publish_setpoint_safe) and slide the
    # commanded motion along the hull's boundary rather than letting it
    # cross into the margin -- this runs every control tick regardless of
    # mission state or whether replanning has caught up.
    #
    # Hard limit: the drone cannot avoid a part of the zone it has never
    # seen. The hull only grows as the camera passes near it, so a zone
    # that has never been glimpsed offers no protection yet. Two ways to
    # harden this further:
    #   1. Fly the first lane pass at a slightly lower altitude / wider
    #      overlap so more ground is seen before any lane is completed.
    #   2. Add a PX4/QGroundControl geofence around the known ARENA bounds
    #      (not the red zone, which isn't known ahead of time) as an
    #      independent hardware-enforced backstop -- see the message text
    #      for the QGC steps.
    # ------------------------------------------------------------------
    def _current_obstacles(self):
        if self.red_zone_hull is not None and len(self.red_zone_hull) >= 3:
            return [np.asarray(self.red_zone_hull, float)]
        return []

    def _safe_velocity(self, pos_ne, vx, vy, margin=None, buffer=None):
        margin = self.obstacle_margin if margin is None else margin
        buffer = self.SAFETY_BUFFER_M if buffer is None else buffer
        v = np.array([vx, vy], float)
        for obs in self._current_obstacles():
            d = float(poly_sdist_many([pos_ne], obs)[0])
            if d < margin + buffer:
                eps = 0.05
                gx = (poly_sdist_many([[pos_ne[0] + eps, pos_ne[1]]], obs)[0]
                     - poly_sdist_many([[pos_ne[0] - eps, pos_ne[1]]], obs)[0]) / (2 * eps)
                gy = (poly_sdist_many([[pos_ne[0], pos_ne[1] + eps]], obs)[0]
                     - poly_sdist_many([[pos_ne[0], pos_ne[1] - eps]], obs)[0]) / (2 * eps)
                n = np.array([gx, gy])
                nn = np.linalg.norm(n)
                if nn < 1e-6:
                    continue
                n /= nn
                if d < margin:
                    # Inside the margin: push out, but scaled by how far in
                    # rather than always slamming to max speed -- the old
                    # unconditional full-speed override fought directly
                    # against an intentional hold during precision landing,
                    # where the vetted target can legitimately sit this
                    # close to a boundary-placed tag.
                    depth = min(1.0, (margin - d) / max(margin, 0.05))
                    v = n * (self.MAX_HORIZ_SPEED * depth)
                else:
                    vn = float(v @ n)
                    if vn < 0:
                        v = v - vn * n                    # slide: remove the inward component only
        return float(v[0]), float(v[1])

    def _safe_position(self, pos_ne, x, y):
        p = np.array([x, y], float)
        for obs in self._current_obstacles():
            d = float(poly_sdist_many([p], obs)[0])
            if d < self.obstacle_margin:
                eps = 0.05
                gx = (poly_sdist_many([[p[0] + eps, p[1]]], obs)[0]
                     - poly_sdist_many([[p[0] - eps, p[1]]], obs)[0]) / (2 * eps)
                gy = (poly_sdist_many([[p[0], p[1] + eps]], obs)[0]
                     - poly_sdist_many([[p[0], p[1] - eps]], obs)[0]) / (2 * eps)
                n = np.array([gx, gy])
                nn = np.linalg.norm(n)
                if nn > 1e-6:
                    p = p + (self.obstacle_margin - d) * n / nn
        return float(p[0]), float(p[1])

    def _decode_frame(self, msg):
        """sensor_msgs/Image -> (gray, small_bgr, err), built straight from the byte
        buffer (CvBridge asserts on the empty frames some camera publishers emit
        at startup). Only a full-res gray and a downscaled BGR copy for the
        red-zone scan are produced -- no full-res colour conversion."""
        h, w, step = int(msg.height or 0), int(msg.width or 0), int(msg.step or 0)
        enc = str(msg.encoding or '').lower()
        ch = {'rgb8': 3, 'bgr8': 3, 'mono8': 1, '8uc1': 1}.get(enc)
        info = f"encoding={enc} {w}x{h} step={step}"
        if h < 2 or w < 2:
            return None, None, f"invalid dimensions ({info})"
        if ch is None:
            return None, None, f"unsupported encoding ({info})"
        row = w * ch
        if step < row:
            return None, None, f"step < row bytes ({info})"
        try:
            raw = np.frombuffer(msg.data, dtype=np.uint8)
        except (TypeError, ValueError) as e:
            return None, None, f"unreadable data: {e} ({info})"
        if raw.size < h * step:
            return None, None, f"short buffer: {raw.size} < {h * step} ({info})"
        px = np.ascontiguousarray(raw[:h * step].reshape(h, step)[:, :row]).reshape(h, w, ch)
        if ch == 1:
            gray = px.reshape(h, w)
        else:
            gray = cv2.cvtColor(px, cv2.COLOR_RGB2GRAY if enc == 'rgb8' else cv2.COLOR_BGR2GRAY)
        s = min(1.0, self.SEARCH_MAX_WIDTH / w)
        small = cv2.resize(px, None, fx=s, fy=s, interpolation=cv2.INTER_AREA) if s < 1.0 else px
        if ch == 1:
            small = cv2.cvtColor(small, cv2.COLOR_GRAY2BGR)
        elif enc == 'rgb8':
            small = cv2.cvtColor(small, cv2.COLOR_RGB2BGR)
        return gray, small, None

    def _process_red_zone(self, bgr, pos, q, W, H, f):
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        mask = cv2.bitwise_or(cv2.inRange(hsv, self.RED_HSV_LO1, self.RED_HSV_HI1),
                              cv2.inRange(hsv, self.RED_HSV_LO2, self.RED_HSV_HI2))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
        cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not cnts:
            return
        c = max(cnts, key=cv2.contourArea)
        if cv2.contourArea(c) < self.RED_MIN_PIXEL_AREA:
            return
        hull_px = cv2.convexHull(c).reshape(-1, 2)
        if len(hull_px) > 8:
            hull_px = hull_px[np.linspace(0, len(hull_px) - 1, 8).astype(int)]
        new_pts = []
        for u, v in hull_px:
            gp = self._ground_point((float(u), float(v)), pos, q, W, H, f)
            if gp is not None:
                new_pts.append(gp)
        if not new_pts:
            return

        # A QR tag's own material/backdrop/mounting hardware can trigger the
        # same HSV threshold as the painted zone -- and since tags are never
        # actually placed inside the zone, a "red" detection whose points
        # land essentially on top of a known or currently-tracked tag is far
        # more likely to be that tag than genuine zone paint. Drop points
        # too close to any such tag rather than let a false zone balloon
        # around it (which then makes the real tag start failing its own
        # zone-exclusion check -- a self-inflicted false positive).
        exclude_pts = [(k['n'], k['e']) for k in self.known_tags]
        with self.lock:
            if self.target is not None:
                exclude_pts.append((float(self.target[0]), float(self.target[1])))
        if exclude_pts:
            new_pts = [p for p in new_pts
                      if not any(math.hypot(p[0] - e[0], p[1] - e[1]) < self.TAG_EXCLUSION_RADIUS_M
                                for e in exclude_pts)]
        if not new_pts:
            return
        with self.lock:
            # Outlier rejection: a single mistimed pose lookup or a stray
            # reddish pixel elsewhere in frame can otherwise inject one bad
            # ground point that instantly balloons the convex hull (and with
            # it, which cells get blocked). Require a new point to be near
            # something already trusted -- real detections of the same
            # physical zone naturally chain together as the drone scans
            # across it; an isolated one-off does not.
            accepted = list(self.red_pts)
            for p in new_pts:
                near_existing = any(math.hypot(p[0] - q[0], p[1] - q[1]) < self.RED_OUTLIER_LINK_M
                                    for q in accepted[-40:])           # recent points are enough to check against
                if near_existing or len(accepted) < self.RED_BOOTSTRAP_PTS:
                    accepted.append(p)
                    self.red_pts.append(p)
            hull = convex_hull(self.red_pts)
            area = poly_area(hull) if len(hull) >= 3 else 0.0
            # Safety-critical: the hull driving _safe_velocity/_safe_position
            # must never lag behind what's actually been seen, so this always
            # updates immediately -- growth-gating below governs only when to
            # trigger a (comparatively expensive, thrash-prone) route replan,
            # never how current the obstacle estimate itself is.
            self.red_zone_hull = hull
            self.red_zone_area = area
            if area > self._replanned_area + self.RED_GROWTH_TRIGGER_M2:
                self._need_replan = True
        if area > 0 and self._need_replan:
            self.get_logger().info(f"Red zone estimate now ~{area:.1f} m^2 ({len(hull)} hull pts)",
                                   throttle_duration_sec=2.0)

    # ------------------------------------------------------------------
    # Target estimate + memory of visited tags
    # ------------------------------------------------------------------
    def _target_snapshot(self):
        with self.lock:
            tgt = None if self.target is None else self.target.copy()
            return tgt, self.target_confirmed, self.last_seen_t

    def _reset_target(self):
        with self.lock:
            self.target = None
            self.n_consistent = 0
            self.reject_streak = 0
            self.target_confirmed = False

    def _is_known(self, ne):
        """Pre-filter only the tag we just finished, for a short cooldown.

        IMPORTANT: historical QR positions are NOT a hard rejection here.
        Ground-point estimates from a downward camera can shift with altitude,
        attitude, and pose interpolation. Rejecting every candidate inside a
        1.2 m historical radius can therefore hide genuinely new QR codes.
        Once a QR is decoded, payload-based dedup in _finish_visit() is the
        authoritative identity check.
        """
        with self.lock:
            pursuit = self.pursuit_ne
            recent = self.last_completed_ne
            recent_t = self.last_completed_t
        if pursuit is not None and math.hypot(ne[0] - pursuit[0], ne[1] - pursuit[1]) < self.PURSUIT_EXEMPT_RADIUS_M:
            return False
        if recent is not None and (self._now_s() - recent_t) < self.RECENT_TAG_SUPPRESS_S:
            return math.hypot(ne[0] - recent[0], ne[1] - recent[1]) < self.RECENT_TAG_SUPPRESS_RADIUS_M
        return False

    def _update_target(self, mx, my, height, now):
        m = np.array([mx, my])
        with self.lock:
            if self.target is None:
                self.target, self.n_consistent, self.reject_streak = m, 1, 0
            else:
                gate = self.GATE_BASE_M + self.GATE_PER_M * height
                if np.linalg.norm(m - self.target) <= gate:
                    a = self.EMA_ALPHA
                    self.target = (1 - a) * self.target + a * m
                    self.n_consistent += 1
                    self.reject_streak = 0
                else:
                    self.reject_streak += 1
                    if self.target_confirmed and self.reject_streak < 4:
                        return
                    self.target, self.n_consistent, self.reject_streak = m, 1, 0
                    self.target_confirmed = False
            if self.n_consistent >= self.CONFIRM_COUNT:
                self.target_confirmed = True
            self.last_seen_t = now

    def _save_results(self):
        try:
            tmp = self.results_file + '.tmp'
            with open(tmp, 'w') as f:
                json.dump(self.known_tags, f, indent=2)
            os.replace(tmp, self.results_file)
        except Exception as e:                           # noqa: BLE001
            self.get_logger().warn(f"Could not write {self.results_file}: {e}")

    def _record_tag(self, ne, payload, status):
        rec = {"id": len(self.known_tags) + 1, "payload": payload, "status": status,
               "n": float(ne[0]), "e": float(ne[1]), "sim_time_s": round(self._now_s(), 1)}
        with self.lock:
            self.known_tags.append(rec)
            self.known_positions.append((float(ne[0]), float(ne[1])))
        self._save_results()
        return rec

    # ------------------------------------------------------------------
    # QR detection + decoding
    # ------------------------------------------------------------------
    def _detect_qr(self, gray, retry):
        variants = [gray]
        if retry:
            variants.append(self.clahe.apply(gray))
        for img in variants:
            for _, det in self.detectors:
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

    def _try_decode(self, raw, best):
        corners = best['corners']
        H, W = raw.shape
        x0, y0 = corners.min(axis=0)
        x1, y1 = corners.max(axis=0)
        pad = 0.35 * max(x1 - x0, y1 - y0)
        xa, xb = int(max(0, x0 - pad)), int(min(W, x1 + pad))
        ya, yb = int(max(0, y0 - pad)), int(min(H, y1 + pad))
        crop = np.ascontiguousarray(raw[ya:yb, xa:xb])
        if crop.size == 0:
            return None
        variants = [crop, self.clahe.apply(crop)]
        if max(crop.shape) < 500:
            variants.append(cv2.resize(crop, None, fx=2, fy=2, interpolation=cv2.INTER_CUBIC))
        variants.append(cv2.threshold(crop, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)[1])
        for img in variants:
            if _zbar_decode is not None:
                res = _zbar_decode(img)
                if res:
                    return res[0].data.decode('utf-8', 'replace')
            for _, det in self.detectors:
                try:
                    txt, _, _ = det.detectAndDecode(img)
                except cv2.error:
                    continue
                if txt:
                    return txt
        return None

    def _is_done(self, m, corners, raw):
        """True if this candidate is a tag we've already read."""
        with self.lock:
            pursuit = self.pursuit_ne
            near = any(math.hypot(m[0] - p[0], m[1] - p[1]) < self.DONE_RADIUS_M for p in self.done_ne)
            seen = {k['payload'] for k in self.known_tags}
        if not near or (pursuit is not None
                        and math.hypot(m[0] - pursuit[0], m[1] - pursuit[1]) < self.PURSUIT_EXEMPT_RADIUS_M):
            return False
        # ponytail: an undecodable new tag within DONE_RADIUS_M of a read one is
        # skipped until some frame decodes it; shrink DONE_RADIUS_M if tags are packed tighter.
        text = self._try_decode(raw, {'corners': corners})
        return text is None or text in seen

    def _evaluate_candidates(self, cands, expected_px, ref_uv, pos, q, W, H, f, raw):
        lo, hi = self.SIZE_GATE
        ref = ref_uv if ref_uv is not None else (W / 2.0, H / 2.0)
        best, best_d = None, 1e18
        with self.lock:
            pursuit = self.pursuit_ne
        for c in cands:
            side = float(np.mean(np.linalg.norm(c - np.roll(c, -1, axis=0), axis=1)))
            if not (lo <= side / expected_px <= hi):
                self.get_logger().info(
                    f"Candidate dropped: size ratio {side / expected_px:.2f} outside {self.SIZE_GATE}",
                    throttle_duration_sec=2.0)
                continue
            cen = c.mean(axis=0)
            m = self._ground_point((cen[0], cen[1]), pos, q, W, H, f)
            if m is None:
                self.get_logger().info("Candidate dropped: no ground point", throttle_duration_sec=2.0)
                continue
            if self._is_known(m):
                self.get_logger().info("Candidate dropped: recently completed tag", throttle_duration_sec=2.0)
                continue
            if self._is_done(m, c, raw):
                self.get_logger().info("Candidate dropped: already-read tag", throttle_duration_sec=2.0)
                continue
            # The tag being pursued/landed on was already decoded, so it is known
            # to be valid; its ground-point jitter must not let a nearby red-zone
            # hull edge reject it mid-descent.
            on_pursuit = pursuit is not None and math.hypot(m[0] - pursuit[0], m[1] - pursuit[1]) < self.PURSUIT_EXEMPT_RADIUS_M
            if not on_pursuit and self.red_zone_hull is not None and len(self.red_zone_hull) >= 3:
                sd = float(poly_sdist_many([m], np.asarray(self.red_zone_hull, float))[0])
                if sd < -0.2:
                    self.get_logger().info(
                        f"Candidate dropped: inside red zone by {-sd:.2f} m at ({m[0]:.1f},{m[1]:.1f})",
                        throttle_duration_sec=2.0)
                    continue  # genuinely inside the polygon (not just near the edge) -- tags are never placed there
            d = float(np.hypot(cen[0] - ref[0], cen[1] - ref[1]))
            if d < best_d:
                best, best_d = {"corners": c, "m": m, "height": float(-pos[2])}, d
        return best

    def _ingest(self, best, stage, raw, now):
        self._update_target(best['m'][0], best['m'][1], best['height'], now)
        self.vision_stage = stage
        if self.want_decode and raw is not None and self.decoded is None:
            text = self._try_decode(raw, best)
            if text:
                with self.lock:
                    self.decoded = (text, best['height'])

    def image_callback(self, msg):
        t0 = time.monotonic()
        try:
            now = self._now_s()
            t_img = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            if abs(now - t_img) > 5.0:
                t_img = now
            pose = self._pose_at(t_img - self.POSE_LAG_S)
            if pose is None:
                return
            pos, q = pose
            height = -pos[2]
            if height < self.BLIND_ALT_AGL:
                return

            gray, small_bgr, image_err = self._decode_frame(msg)
            if gray is None:
                self.get_logger().warn(f"Skipping camera frame: {image_err}", throttle_duration_sec=5.0)
                return
            Hf, Wf = gray.shape
            f_full = Wf / (2.0 * np.tan(self.CAM_HFOV_RAD / 2.0))
            f_small = small_bgr.shape[1] / (2.0 * np.tan(self.CAM_HFOV_RAD / 2.0))
            self._process_red_zone(small_bgr, pos, q, small_bgr.shape[1], small_bgr.shape[0], f_small)
            expected_px = f_full * self.TAG_SIZE_M / max(height, 0.5)

            best, stage = None, "NONE"
            est, _, _ = self._target_snapshot()
            if est is not None:
                uv = self._project_world_to_pixel(est, pos, q, Wf, Hf, f_full)
                if uv is not None:
                    best = self._evaluate_candidates(
                        self._detect_in_roi(gray, uv, expected_px), expected_px, uv, pos, q, Wf, Hf, f_full, gray)
                    stage = "ROI" if best else stage
            if best is None:
                best = self._evaluate_candidates(
                    self._detect_full(gray), expected_px, None, pos, q, Wf, Hf, f_full, gray)
                stage = "FULL" if best else stage
            if best is None:
                return
            self._ingest(best, stage, gray, now)
        except Exception as e:                           # noqa: BLE001
            self.get_logger().error(
                f"Vision processing error: {type(e).__name__}: {e}",
                throttle_duration_sec=2.0)

    # ------------------------------------------------------------------
    # Live 2D visualisation snapshot (no effect on flight logic)
    # ------------------------------------------------------------------
    def _write_viz_snapshot(self, now):
        self._trail_tick += 1
        if self._trail_tick % 4 == 0:                    # ~5 Hz trail sampling
            self.trail.append([round(self.current_pos[0], 2), round(self.current_pos[1], 2)])
        self._viz_tick += 1
        if self._viz_tick % 4 != 0:                       # ~5 Hz snapshot writes
            return
        try:
            visited_cells, blocked_cells = self._viz_cell_status() if self.seen is not None else ([], [])
            snap = {
                "t": round(now, 1),
                "state": self.state,
                "target_payload": self.target_payload,
                "pos": [round(self.current_pos[0], 2), round(self.current_pos[1], 2)],
                "height": round(-self.current_pos[2], 2),
                "bounds": [list(p) for p in self.bounds_ne] if self.bounds_ne else [],
                "red_zone": [list(p) for p in self.red_zone_hull] if self.red_zone_hull else [],
                "red_zone_area": round(self.red_zone_area, 1),
                "tags": [{"id": k["id"], "n": k["n"], "e": k["e"], "status": k["status"], "payload": k["payload"]}
                         for k in self.known_tags],
                "route": [[lane, round(pt[0], 2), round(pt[1], 2)] for lane, pt in self.route],
                "trail": list(self.trail),
                "cells": self.viz_cells,
                "cell_size": [self._viz_blk * self.COV_RES] * 2 if self.seen is not None else [0, 0],
                "visited_cells": visited_cells,
                "blocked_cells": blocked_cells,
            }
            tmp = self.viz_file + '.tmp'
            with open(tmp, 'w') as f:
                json.dump(snap, f)
            os.replace(tmp, self.viz_file)
        except Exception:                                # noqa: BLE001
            pass                                          # visualization is best-effort, never blocks flight

    # ------------------------------------------------------------------
    # PX4 I/O
    # ------------------------------------------------------------------
    def get_timestamp_us(self):
        return int(self.get_clock().now().nanoseconds / 1000)

    def publish_offboard_control_mode(self):
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = True
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = self.get_timestamp_us()
        self.offboard_control_mode_publisher.publish(msg)

    def publish_setpoint(self, x=None, y=None, z=None, vx=None, vy=None):
        nan = float('nan')
        msg = TrajectorySetpoint()
        msg.position = [float(x) if x is not None else nan,
                        float(y) if y is not None else nan,
                        float(z) if z is not None else nan]
        msg.velocity = [float(vx) if vx is not None else nan,
                        float(vy) if vy is not None else nan, nan]
        msg.yaw = 0.0
        msg.timestamp = self.get_timestamp_us()
        self.trajectory_setpoint_publisher.publish(msg)

    def publish_setpoint_safe(self, pos, x=None, y=None, z=None, vx=None, vy=None, margin=None, buffer=None):
        """Every horizontal command in the node goes through here -- see the
        RED-ZONE SAFETY note above _current_obstacles(). margin/buffer let a
        caller use a tighter zone than the general sweep/approach one --
        PRECISION_LAND does, since it's a slow, deliberate approach to a
        point already vetted as outside the polygon, not open exploration."""
        # Tags can't be inside the red zone, so near a tag the filter can only
        # block the read/landing (e.g. a tag beside the zone's edge). APPROACH
        # is deliberately NOT exempt: it still gets the red-zone filter.
        if self.state in ("READ", "PRECISION_LAND", "ASCEND"):
            self.publish_setpoint(x=x, y=y, z=z, vx=vx, vy=vy)
        elif vx is not None or vy is not None:
            vx2, vy2 = self._safe_velocity((pos[0], pos[1]), vx or 0.0, vy or 0.0, margin=margin, buffer=buffer)
            self.publish_setpoint(z=z, vx=vx2, vy=vy2)
        elif x is not None and y is not None:
            xs, ys = self._safe_position((pos[0], pos[1]), x, y)
            self.publish_setpoint(x=xs, y=ys, z=z)
        else:
            self.publish_setpoint(x=x, y=y, z=z)

    def send_vehicle_command(self, command, param1=0.0, param2=0.0):
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = float(param1)
        msg.param2 = float(param2)
        msg.param3 = msg.param4 = msg.param5 = msg.param6 = msg.param7 = 0.0
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = self.get_timestamp_us()
        self.vehicle_command_publisher.publish(msg)

    # ------------------------------------------------------------------
    # Visit handling
    # ------------------------------------------------------------------
    def _begin_visit(self, tgt, pos):
        self.visit_t0 = self._now_s()
        self.decoded = None
        self.want_decode = True
        self._centered_ticks = 0
        self.floor_t0 = None
        self.hold_ne = (float(tgt[0]), float(tgt[1]))
        self.pursuit_ne = (float(tgt[0]), float(tgt[1]))   # active tag reference during the visit
        self.landing_target_ne = None  # populated only when the actual target is confirmed
        self.approach_route = deque()
        self._approach_replan_t = -1e9
        self.approach_last_tgt = (float(tgt[0]), float(tgt[1]))
        self._reset_progress()
        self.get_logger().info(f"New tag at n={tgt[0]:.1f} e={tgt[1]:.1f} -> approaching")
        self.state = "APPROACH"

    def _reset_progress(self):
        self._progress_best = None
        self._progress_t = self._now_s()

    def _stuck(self, err, now):
        """Cause-agnostic watchdog: true once distance-to-target hasn't
        improved in STUCK_TIMEOUT_S, whatever the reason (sliding along an
        obstacle with no way around it, chasing an unstable/phantom
        detection that drifts as we approach, etc). Reactive-only obstacle
        avoidance can get stuck exactly like this on geometry it has no
        pathfinding to route around, so this exists as the backstop even
        though the APPROACH path is also now planned, not just pursued."""
        if self._progress_best is None or err < self._progress_best - self.PROGRESS_EPS_M:
            self._progress_best, self._progress_t = err, now
            return False
        return (now - self._progress_t) > self.STUCK_TIMEOUT_S

    def _pop_decoded(self):
        with self.lock:
            d, self.decoded = self.decoded, None
        return d

    def _begin_precision_landing(self, ne, pos):
        self.get_logger().info("Beginning precision landing on the target.")
        self.pursuit_ne = self.landing_target_ne = ne
        self.land_lost_t0 = None
        self.z_cmd = pos[2]
        self._reset_progress()
        self.floor_t0 = None
        self.state = "PRECISION_LAND"

    def _go_ascend(self, pos):
        self.pursuit_ne = None
        self.landing_target_ne = None
        self._reset_target()
        self.z_cmd = pos[2]
        self.state = "ASCEND"

    def _finish_visit(self, tgt, payload, status, pos):
        ne = tgt if tgt is not None else np.array(self.hold_ne)
        ne = (float(ne[0]), float(ne[1]))
        dup = None
        if status == "read" and payload is not None:
            with self.lock:
                self.done_ne.append(ne)
            # Payload-based dedup: every physical QR has unique content, so decoded
            # text identifies a tag far better than position (a far glance and a close
            # read can land >1 m apart).
            dup = next((k for k in self.known_tags if k['payload'] == payload), None)

        if dup is not None:
            self.get_logger().info(
                f"'{payload}' already recorded as #{dup['id']} -- same tag re-approached "
                f"from a different angle, not re-recording.")
            with self.lock:
                self.known_positions.append(ne)   # this drifted estimate too
            is_target = self.target_payload is not None and payload == self.target_payload
        else:
            is_target = False
            if status == "read" and self.target_payload is not None:
                is_target = (payload == self.target_payload)
                status = "target_found" if is_target else "read_nonmatch"
            rec = self._record_tag(ne, payload, status)
            if status == "target_found":
                self.get_logger().info(
                    f"*** TARGET FOUND *** '{payload}' at ({rec['n']:.1f}, {rec['e']:.1f}) -- landing.")
            elif status == "read_nonmatch":
                self.get_logger().info(f"Tag #{rec['id']} '{payload}' does not match target, continuing search.")
            elif status == "read":
                self.get_logger().info(f"QR #{rec['id']} READ: '{payload}' at ({rec['n']:.1f}, {rec['e']:.1f})")
            else:
                self.get_logger().warn(f"Tag #{rec['id']} at ({rec['n']:.1f}, {rec['e']:.1f}) -> {status}")

        self.hold_ne = ne
        with self.lock:
            self.last_completed_ne = ne
            self.last_completed_t = self._now_s()
        self.want_decode = False
        if is_target:
            self._begin_precision_landing(ne, pos)
        else:
            self._go_ascend(pos)

    def _resume_sweep(self, pos):
        # Re-route the queued lane goals from the current position; coverage state
        # means ground already seen is never flown again.
        self._reset_target()
        self._replan_sweep(pos)
        self.state = "SWEEP"

    def _finish_mission(self, pos):
        self.want_decode = False
        self.landing_target_ne = None
        found = [k for k in self.known_tags if k['status'] == 'target_found']
        if self.target_payload is not None:
            self.get_logger().info(
                f"=== SEARCH COMPLETE: target {'FOUND' if found else 'NOT FOUND'} "
                f"({len(self.known_tags)} tag(s) checked) ===")
        else:
            reads = [k for k in self.known_tags if k['status'] == 'read']
            self.get_logger().info(f"=== SWEEP COMPLETE: {len(reads)} QR code(s) read ===")
        for k in self.known_tags:
            self.get_logger().info(f"  #{k['id']} {k['status']:<14} '{k['payload']}' n={k['n']:.1f} e={k['e']:.1f}")
        self.get_logger().info(f"Results saved to {self.results_file}")
        self.send_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        self.state = "DONE"

    # ------------------------------------------------------------------
    # Control loop
    # ------------------------------------------------------------------
    def timer_callback(self):
        ts = self.get_timestamp_us()
        if ts == 0:
            self.get_logger().warn("Clock is at 0! Check ros_gz_bridge for /clock topic.", throttle_duration_sec=2.0)
            return
        if not self.area_ok:
            self.get_logger().error("Refusing to fly: no valid bounds JSON loaded.", throttle_duration_sec=3.0)
            return

        self.publish_offboard_control_mode()
        self._write_viz_snapshot(now=self._now_s())

        now = self._now_s()

        # Mode watchdog: if we're well past the initial arm/offboard sequence
        # but PX4 reports we're no longer actually in OFFBOARD -- a comms
        # blip, a PX4-side failsafe, anything -- the drone is sitting in
        # whatever PX4's own failsafe put it in (usually a hold) regardless
        # of what our internal state machine thinks is happening. Logging
        # this loudly is the main point (this is probably why it "stopped"),
        # and re-requesting OFFBOARD is a cheap, safe thing to also try: PX4
        # only re-enters offboard on an explicit mode request after it's
        # fallen out, not just because setpoints resume arriving.
        if (self.offboard_counter > 30 and self.vehicle_armed
                and self.vehicle_nav_state is not None
                and self.vehicle_nav_state != VehicleStatus.NAVIGATION_STATE_OFFBOARD):
            self.get_logger().error(
                f"PX4 is NOT in OFFBOARD mode (nav_state={self.vehicle_nav_state}) -- it has likely "
                f"fallen back to a failsafe (hold/etc) and is ignoring our setpoints. Check the "
                f"offboard link (MicroXRCEAgent / network) for drops.", throttle_duration_sec=2.0)
            if now - self._last_offboard_reclaim_t > 2.0:
                self._last_offboard_reclaim_t = now
                self.send_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)

        pos = self.current_pos
        height = -pos[2]
        tgt, confirmed, last_seen = self._target_snapshot()
        age = now - last_seen
        dt = self.CONTROL_DT
        self._mark_seen(pos)

        if self.state in ["INIT", "TAKEOFF"]:
            self.publish_setpoint_safe(pos, x=0.0, y=0.0, z=self.SWEEP_Z)

        elif self.state == "SWEEP":
            if self._need_replan and (now - self._last_replan_t) > self.REPLAN_COOLDOWN_S:
                self._need_replan = False
                self._last_replan_t = now
                self._replanned_area = self.red_zone_area
                self._replan_sweep(pos)
            if confirmed and age < self.LOST_TIMEOUT_S:
                self._begin_visit(tgt, pos)
            elif not self._follow_route(pos):
                self.get_logger().info("Coverage finished, returning home.")
                back = self.planner.path((pos[0], pos[1]), (0.0, 0.0)) or [(0.0, 0.0)]
                self.route = deque([(-1, tuple(x)) for x in back])
                self.state = "RETURN"

        elif self.state == "APPROACH":
            dec = self._pop_decoded()
            if dec:
                self._finish_visit(tgt, dec[0], "read", pos)
                return
            # Every confirmed detection in this world corresponds to a real,
            # readable tag -- so a tracking gap is a hiccup to ride out, not
            # a reason to give up. The only hard stop is the overall timeout;
            # a momentarily stale tgt just means we keep heading toward the
            # last position we actually had for it, instead of abandoning.
            if tgt is not None:
                self.approach_last_tgt = (float(tgt[0]), float(tgt[1]))
            nav_tgt = self.approach_last_tgt
            if nav_tgt is None or now - self.visit_t0 > self.APPROACH_TIMEOUT_S:
                self.get_logger().warn(
                    f"Approach timed out after {self.APPROACH_TIMEOUT_S:.0f}s without a usable read.")
                self._finish_visit(tgt, None, "approach_timeout", pos)
                return
            err = float(np.hypot(nav_tgt[0] - pos[0], nav_tgt[1] - pos[1]))
            if self._stuck(err, now):
                self.get_logger().warn(
                    f"No progress toward tag for {self.STUCK_TIMEOUT_S:.0f}s (stuck at {err:.1f}m) -- "
                    f"likely boxed in by the red zone; marking unreachable for now.")
                self._finish_visit(tgt, None, "unreachable", pos)
                return

            # Routed approach, not raw pursuit: a straight line to the target
            # can run straight into the red zone, and the reactive safety
            # filter alone can only slide along its edge, not route around
            # it -- that combination is exactly what produced the "stuck
            # sliding, then off in one direction forever" failure. Refreshed
            # periodically since the target estimate keeps refining as we
            # get closer.
            if (not self.approach_route) or (now - self._approach_replan_t > self.APPROACH_REPLAN_S):
                self.approach_route = deque([nav_tgt])   # straight in: tags are never in the red zone
                self._approach_replan_t = now
            p = np.array([pos[0], pos[1]])
            while self.approach_route and np.linalg.norm(np.array(self.approach_route[0]) - p) < self.WP_TOL:
                self.approach_route.popleft()
            if self.approach_route:
                d = np.array(self.approach_route[0]) - p
                dist = float(np.linalg.norm(d))
                speed = min(self.MAX_HORIZ_SPEED, max(0.3, self.ALIGN_KP * dist))
                v = d / max(dist, 1e-6) * speed
                self.publish_setpoint_safe(pos, z=self.SWEEP_Z, vx=float(v[0]), vy=float(v[1]))
            else:
                # Routed path fully consumed (within WP_TOL=0.8m of nav_tgt),
                # but that's looser than APPROACH_TOL=0.4m -- commanding zero
                # here left a dead zone where nothing ever closed the last
                # bit of gap needed to start reading. Direct pursuit for
                # this final stretch instead; we're already essentially at
                # the target, so routing doesn't matter anymore.
                ex2, ey2 = nav_tgt[0] - pos[0], nav_tgt[1] - pos[1]
                self.publish_setpoint_safe(pos, z=self.SWEEP_Z,
                                           vx=self._clamp(self.ALIGN_KP * ex2, self.MAX_HORIZ_SPEED),
                                           vy=self._clamp(self.ALIGN_KP * ey2, self.MAX_HORIZ_SPEED))

            self._centered_ticks = self._centered_ticks + 1 if (err < self.APPROACH_TOL and age < 1.0) else 0
            if self._centered_ticks >= self.APPROACH_TICKS:
                self.z_cmd = pos[2]
                self.floor_t0 = None
                self._reset_progress()
                self.state = "READ"

        elif self.state == "READ":
            dec = self._pop_decoded()
            if dec:
                self._finish_visit(tgt, dec[0], "read", pos)
                return
            if tgt is None or age > self.READ_ABORT_S:
                self._finish_visit(tgt, None, "lost", pos)
                return
            ex, ey = tgt[0] - pos[0], tgt[1] - pos[1]
            err = float(np.hypot(ex, ey))
            if self._stuck(err, now):
                self.get_logger().warn(f"No progress centering to read for {self.STUCK_TIMEOUT_S:.0f}s: giving up on this tag.")
                self._finish_visit(tgt, None, "lost", pos)
                return
            allowed = max(0.15, 0.10 * height)
            if err < allowed and age < self.LOST_TIMEOUT_S:
                self.z_cmd += self.READ_DESCENT_RATE * dt
            self.z_cmd = min(self.z_cmd, -self.READ_MIN_ALT, pos[2] + 0.5)
            self.publish_setpoint_safe(pos, z=self.z_cmd,
                                       vx=self._clamp(self.ALIGN_KP * ex, self.MAX_HORIZ_SPEED),
                                       vy=self._clamp(self.ALIGN_KP * ey, self.MAX_HORIZ_SPEED))
            if height <= self.READ_MIN_ALT + 0.15:
                self.floor_t0 = self.floor_t0 if self.floor_t0 is not None else now
                if now - self.floor_t0 > self.READ_FLOOR_DWELL_S:
                    self._finish_visit(tgt, None, "unreadable", pos)

        elif self.state == "PRECISION_LAND":
            # IMPORTANT: do NOT use self.target as the landing reference here.
            # With a downward-facing camera, once the QR is centered its
            # projected ground point is approximately the drone's CURRENT
            # position. Feeding that back into self.target makes the target
            # appear to move with the vehicle, so err collapses toward 0.
            # Freeze the target world coordinate at the moment we confirmed
            # its payload, then use that fixed point for the entire descent.
            landing_tgt = self.landing_target_ne or self.pursuit_ne or self.hold_ne
            ex = float(landing_tgt[0] - pos[0])
            ey = float(landing_tgt[1] - pos[1])
            err = float(np.hypot(ex, ey))

            # NAV_LAND drops straight down from wherever it's called, so only
            # hand off once centred; if centring never settles, give up after
            # LAND_CENTER_WAIT_S rather than hover forever.
            if height <= self.FINAL_ALT_AGL:
                self.floor_t0 = self.floor_t0 if self.floor_t0 is not None else now
                if err < self.LAND_CENTER_TOL or now - self.floor_t0 > self.LAND_CENTER_WAIT_S:
                    self.get_logger().info(f"Touchdown altitude reached (err={err:.2f}m), commanding LAND/disarm.")
                    self._finish_mission(pos)
                    return

            # `last_seen` is still updated by vision for diagnostics, but it is
            # NOT allowed to replace the frozen landing target. A short vision
            # dropout therefore holds the last known position instead of
            # snapping err to zero or chasing a moving projection.
            tag_ok = age < self.LOST_TIMEOUT_S or height < self.LAND_BLIND_ALT_AGL

            # Tighter gate and slower descent as the ground nears: lateral error
            # that was fine at 3 m is a miss at 0.5 m.
            allowed = max(0.08, 0.05 * height)
            if tag_ok:
                self.land_lost_t0 = None
            else:
                self.land_lost_t0 = self.land_lost_t0 if self.land_lost_t0 is not None else now
                if now - self.land_lost_t0 > self.LAND_LOST_GRACE_S:
                    # Tag gone and we're too high to descend blind: climb over the
                    # frozen target until it's seen again (tag_ok flips back and the
                    # descent resumes), or give up at sweep altitude and resume sweep.
                    if pos[2] <= self.SWEEP_Z + 0.3:
                        self.get_logger().warn("Lost tag during PRECISION_LAND, could not re-acquire; resuming sweep.")
                        self.land_lost_t0 = None
                        self._go_ascend(pos)
                        return
                    self.z_cmd = max(self.z_cmd - self.CLIMB_RATE * dt, self.SWEEP_Z, pos[2] - 1.0)
            if err < allowed and tag_ok:
                self.z_cmd += self.READ_DESCENT_RATE * min(1.0, max(0.3, height / 2.0)) * dt

            # Never command a horizontal point farther than 0.5 m from the
            # current vehicle position in one tick; this limits a stale/frozen
            # target to a bounded correction rather than a jump.
            self.z_cmd = min(self.z_cmd, pos[2] + 0.5)
            self.publish_setpoint_safe(
                pos, z=self.z_cmd,
                vx=self._clamp(self.ALIGN_KP * ex, self.MAX_HORIZ_SPEED),
                vy=self._clamp(self.ALIGN_KP * ey, self.MAX_HORIZ_SPEED),
                margin=self.PRECISION_MARGIN_M, buffer=self.PRECISION_BUFFER_M)

            self.get_logger().info(
                f"PRECISION_LAND h={height:.2f}m err={err:.2f}m "
                f"(allowed {allowed:.2f}, tag_ok={tag_ok}) seen {age:.1f}s ago "
                f"target=({landing_tgt[0]:.2f},{landing_tgt[1]:.2f})",
                throttle_duration_sec=1.0)

        elif self.state == "ASCEND":
            self.z_cmd = max(self.z_cmd - self.CLIMB_RATE * dt, self.SWEEP_Z, pos[2] - 1.0)
            self.publish_setpoint_safe(pos, x=self.hold_ne[0], y=self.hold_ne[1], z=self.z_cmd)
            if pos[2] <= self.SWEEP_Z + 0.3:
                self._resume_sweep(pos)

        elif self.state == "RETURN":
            if not self._follow_route(pos, plan_more=False):
                self._finish_mission(pos)

        elif self.state == "DONE":
            return

        if self.offboard_counter < 15:
            self.offboard_counter += 1
            return
        if self.offboard_counter == 15:
            self.get_logger().info("Requesting Offboard mode...")
            self.send_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
        elif self.offboard_counter == 25:
            self.get_logger().info("Requesting Arming...")
            self.send_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
        elif self.offboard_counter == 30:
            self.get_logger().info("OFFBOARD & ARM commands dispatched! Executing climb.")
            self.state = "TAKEOFF"

        if self.state == "TAKEOFF":
            self.takeoff_ticks += 1
            if self.takeoff_ticks % 20 == 0:
                self.get_logger().info(f"Climbing... Height: {height:.2f}m")
            if pos[2] <= self.SWEEP_Z + 0.5 or self.takeoff_ticks >= 160:
                self.get_logger().info("Sweep altitude reached, planning coverage.")
                self._start_sweep(pos)

        self.offboard_counter += 1


def main(args=None):
    rclpy.init(args=args)
    node = PX4QRSweepNode()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()