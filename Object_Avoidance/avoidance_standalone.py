#!/usr/bin/env python3
"""
AVOIDANCE_STANDALONE - one-file build of the corridor obstacle-avoidance node (no Gazebo, no colcon package).

Contains: the lateral avoidance node (CorridorFlight) + the vertical over/under layer (VerticalPlanner), with the
validated narrow-corridor settings (corridor_flight_v00_rm05.yaml) baked in as defaults for a 1.5-1.8 m corridor.

RUN (Jetson, ROS 2 Humble sourced, px4_msgs built, camera driver running):
  1. BENCH / DRY RUN (default, props OFF): perceives and prints decisions, sends NOTHING to the flight controller
         python3 avoidance_standalone.py
     Watch  ros2 topic echo /depth_navigation/decision   and   /depth_navigation/diagnostics
  2. FLIGHT (node arms, takes off to takeoff_altitude, then flies the corridor):
         python3 avoidance_standalone.py --ros-args -p flight_enabled:=true
  Any parameter can be overridden:  --ros-args -p max_speed:=0.4 -p depth_topic:=/camera/depth/image_raw

REAL-DRONE CHECKLIST (what differs from the Gazebo runs)
  * depth_topic   default /camera/depth/image_raw (Orbbec ROS 2 driver). Node accepts 32FC1 (m) and 16UC1 (mm).
  * hfov_deg      86 deg was used in simulation. CHECK the real Gemini 335 depth HFOV (datasheet / camera_info) and set it.
  * takeoff_altitude 2.0 m = cruise height above the floor; vert_ground_offset is 0.0 (the 0.48 m of the sim
                  only existed because the simulated origin was the settled spawn height).
  * max_speed     0.4 m/s default. All validation in Gazebo was done at 0.4 m/s. Do NOT raise it for first flights:
                  collisions occurred at 1-2 m/s in the dense gauntlet.
  * WALL CENTRING works in a 1.65 m corridor (after a dodge the drone returns to within ~0.1 m of the centre line).
                  /depth_navigation/diagnostics shows wallL/wallR and cgate= (why centring is idle: avoiding, probe, ok ...).
                  Tuning keys: center_probe_dist (1.0), centering_extra_lat (-0.20), center_deadband (0.15).
  * KNOWN LIMITS (tested in Gazebo, 1.65 m corridor, 0.4 m/s; the drone stops safely, it does not collide):
                  slalom layouts with ~4 m spacing and flush full-height blocks (gap ~1.07 m for a 0.67 m body),
                  and some randomised full-gauntlet layouts, can end in STUCK / HOLD (a safe stall). R10 passes about half
                  of the runs for the same reason (a 1.07 m gap beside a flush block).
                  Over/under passes (beam, bar, window, low wall, 12 m roof) are handled; nothing collided in any run.
  * flight_enabled defaults to FALSE (dry run).
  * Interface is PX4 offboard (px4_msgs, /fmu/in/..., /fmu/out/...). An ArduPilot flight controller needs an adapter
    for the 4 PX4 topics (position, status, setpoint, vehicle command); the avoidance logic itself is unchanged.

---- original description ----
CorridorFlight - depth-camera obstacle avoidance for PX4 (ROS 2 + Gazebo SITL)

Pipeline
--------
depth image (z-depth) -> radial range -> 31 sectors -> clearance per sector
-> no-data blobs get a range estimated from apparent size
-> short-term obstacle memory (EKF position) for obstacles beside the drone
-> per-heading free distance (path within safety_radius of any remembered/seen point)
-> instant-block / delayed-release -> choose best free bearing (VFH-style cost)
-> avoidance latch + wall-centering -> body velocity -> local NED
-> rate-limited offboard velocity setpoint

Cases handled (each is exercised by test_cases.py / sim.py)
-----------------------------------------------------------
Perception
* z-depth -> radial range; float32 (m) or uint16 (mm); bad / truncated / tiny frames -> hold
* thin poles caught by a low percentile; sensor dropouts (speckle) ignored
* no-data columns (black / matte / out of range): range <= nodata_width / (2 tan(alpha/2));
  very wide no-data areas (sky, blank wall) stay "unknown": blocked, never approached
Decision
* obstacle ahead -> best free heading (goal, smoothness, openness); range-dependent clearance
* centre heading closer than stop_distance -> pure lateral escape, only AWAY from the
  nearest blocker, only toward a side with lateral room (walls / pillars remembered)
* escape side is sticky (no left/right dithering)
* nothing free -> slow creep along the most open known heading, else HOLD;
  HOLD for stuck_timeout -> threshold relaxes to a hard floor; STUCK is reported on
  /depth_navigation/decision; optional slow retreat (off by default)
* pillar leaving the field of view -> distance-based latch keeps the lateral offset until
  the whole drone has passed; obstacle memory (20 s) covers obstacles beside the drone
* wall-centering from left/right clearance
* [REV AeroTHON] wall-vs-obstacle heuristic (see _classify_walls): centering only uses
  long, straight, heading-parallel runs of sectors; anything else stays an obstacle for
  the planner. Centering is suspended during avoidance and ramps back in smoothly.
* [REV AeroTHON] depth-confidence gate: no returns / mostly unknown -> HOLD, partially
  unknown -> speed factor, resume only after N consecutive good frames. Unknown space is
  never "free", also not while the obstacle threshold is relaxed by stuck recovery.
* [REV AeroTHON] conservative deadlock recovery: STOP -> back off ONLY along the path
  already flown (breadcrumb trail) -> re-evaluate; capped number of attempts.
* [REV AeroTHON] dynamic obstacles: urgency-scaled lateral escape, ghost-clearing and
  non-expiring-while-near obstacle memory.
* [REV2] invalid depth: NaN, +inf, -inf, 0 and negative values are all UNKNOWN (never
  free, never converted to max_depth); partly-unknown sectors are capped conservatively.
* [REV2] hold_yaw is established from the real heading before obstacle memory is used.
* [REV2] wall centering: yaw-consistency gate, outlier rejection, low-pass, never toward
  unknown / non-wall obstacles / remembered points; all sectors stay in collision checks.
* [REV2] sticky avoidance side for a blocked centre path (no LEFT/RIGHT dithering);
  recovery adds a guarded lateral ESCAPE step and a forward-progress trigger.
Control / safety
* stale depth, stale/invalid pose -> horizontal hold; exception in the control loop keeps
  the offboard heartbeat alive with a zero-velocity hold
* pilot leaves OFFBOARD or vehicle disarms -> controller aborts (never grabs control back)
* fast braking and reversals (max_decel) vs gentle acceleration
* invalid parameters corrected with a warning; ints accepted for float parameters

Known limits (need real-hardware checks)
* black obstacles that return no depth: range is only estimated; calibrate nodata_width
* sun / IR washout, glossy surfaces, floor or ceiling inside the vertical band
* blind to obstacles never seen (e.g. beside the drone at start)

Tuning / pre-flight checklist (competition)
-------------------------------------------
1. Props OFF, depth live: check /depth_navigation/diagnostics. Open space / lens covered must
   give hold=1. A wall at 1.0 m must give an escape or HOLD, never FORWARD.
2. Set hfov_deg, corridor_width, max_speed (0.4 m/s is the safe default; the design report's
   0.8 m/s is NOT used) and obstacle/stop distances for the real Gemini 330 and corridor.
3. Check nodata_width / percentile / max_unknown_fraction on real depth (IR speckle, glossy walls).
4. Verify yaw stays within ~8 deg of the corridor axis, else wall centering is disabled.
5. Verify EKF eph < pose_max_eph in the hall; otherwise obstacle memory is disabled.
6. Run SITL: pillar, slalom, two obstacles, dead end, sudden obstacle at 2 m / 1 m / 0.5 m.
   An offline logic test (no ROS needed) is provided in test_corridor_flight.py.

Flight safety
-------------
flight_enabled defaults to FALSE. Enable with:
    ros2 run <pkg> corridor_flight --ros-args -p flight_enabled:=true
"""

import math

import numpy as np
import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleLocalPosition,
    VehicleStatus,
)
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray, String

S_WAIT = 'WAIT_SENSORS'
S_PREFLIGHT = 'PREFLIGHT'
S_ARMING = 'ARMING'
S_TAKEOFF = 'TAKEOFF'
S_NAVIGATE = 'NAVIGATE'
S_ABORTED = 'ABORTED'

# Deadlock-recovery sub-states (inside S_NAVIGATE)
R_STOP = 'STOP'
R_BACK = 'BACKOFF'
R_EVAL = 'EVAL'
R_ESC = 'ESCAPE'


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


# ======================================================================
# [VERT] VERTICAL MANOEUVRE LAYER (embedded from vertical_layer.py; pure numpy)
# HEIGHT CONVENTION: metres ABOVE THE FLOOR. alt_agl = -local_ned_z + vert_ground_offset
# ======================================================================
class VerticalPlanner:
    DEFAULTS = dict(
        alt_min=0.6,          # lowest allowed centre height (m AGL)
        alt_max=2.1,          # highest allowed centre height; keep body top below the lowest wall (2.44 m)
        half_h=0.30,          # slab half height = body half height (~0.15) + safety margin (~0.15)
        path_halfwidth=0.45,  # lateral half width of the path checked (drone radius 0.336 + margin)
        look_ahead=4.0,       # only obstacles closer than this (z-depth, m) are considered
        trigger_dist=3.5,     # start a manoeuvre when the slab is blocked closer than this
        min_start_dist=1.0,   # never START a manoeuvre with the blocker closer than this (too late; lateral logic/stop handles it)
        floor_excl=0.15,      # points lower than this are floor (depth noise), never obstacles
        min_pixels=8,         # fewer blocking pixels than this = speckle, ignored
        release_margin=1.0,   # keep the manoeuvre altitude this far (m of travel) beyond the last sighting
        level_step=0.05,      # altitude grid
        mask_margin=0.10,     # mask pixels farther than slab + this from the slab
        min_depth=0.15,
        hfov_deg=86.0,
    )

    def __init__(self, home_agl, **kw):
        self.p = dict(self.DEFAULTS)
        self.p.update(kw)
        self.home = float(home_agl)
        self.target = float(home_agl)
        self.hold_travel = 0.0          # metres still to travel before returning to home
        self.hold_active = False
        self.last_t = None
        self.status = 'cruise'
        self._shape = None
        n = int(round((self.p['alt_max'] - self.p['alt_min']) / self.p['level_step'])) + 1
        self.levels = np.linspace(self.p['alt_min'], self.p['alt_max'], n)

    # ---------------------------------------------------------------- geometry
    def _geometry(self, H, W):
        if self._shape == (H, W):
            return
        fx = (W / 2.0) / math.tan(math.radians(self.p['hfov_deg']) / 2.0)
        u = np.arange(W, dtype=np.float32)
        v = np.arange(H, dtype=np.float32)
        self.xcoef = ((u - (W - 1) / 2.0) / fx).astype(np.float32)[None, :]     # lateral per metre of depth
        self.ycoef = (-(v - (H - 1) / 2.0) / fx).astype(np.float32)[:, None]    # height per metre of depth
        self.tan_v = 0.9 * (H / 2.0) / fx          # vertical half field of view (tangent), 10 % safety
        self._shape = (H, W)

    def _heights(self, depth, alt):
        self._geometry(*depth.shape)
        valid = np.isfinite(depth) & (depth > self.p['min_depth'])
        d = np.where(valid, depth, 0.0).astype(np.float32)
        return valid, d, alt + self.ycoef * d, self.xcoef * d

    # ---------------------------------------------------------------- analysis
    def blockers(self, depth, alt):
        """Heights and distances of every in-path obstacle point (floor excluded)."""
        valid, d, h, lat = self._heights(depth, alt)
        m = valid & (np.abs(lat) <= self.p['path_halfwidth']) & (d <= self.p['look_ahead']) \
            & (h > self.p['floor_excl'])
        return h[m], d[m]

    def level_block_dist(self, hs, ds, a):
        """Distance to the nearest obstacle overlapping the slab centred at a (inf = free)."""
        sel = np.abs(hs - a) <= self.p['half_h']
        if int(np.count_nonzero(sel)) < self.p['min_pixels']:
            return math.inf
        return float(np.percentile(ds[sel], 5))

    def _guard_dist(self, hs, ds):
        """While a manoeuvre is held: distance to the nearest in-path point in the height band that lies BETWEEN the
        held slab and the cruise slab (inf = nothing there). Returning to cruise through that band would hit it."""
        if self.target == self.home or hs.size == 0:
            return math.inf
        hh = self.p['half_h']
        if self.target < self.home:
            lo, hi = self.target + hh, self.home + hh        # under: roof / beam above the lowered slab
        else:
            lo, hi = self.home - hh, self.target - hh        # over: platform / block below the raised slab
        sel = (hs > lo) & (hs <= hi)
        if int(np.count_nonzero(sel)) < self.p['min_pixels']:
            return math.inf
        return float(np.percentile(ds[sel], 5))

    # ---------------------------------------------------------------- update
    def update(self, depth, alt, v_fwd, now):
        """Returns the target altitude (m AGL)."""
        dt = 0.0 if self.last_t is None else max(0.0, min(now - self.last_t, 0.5))
        self.last_t = now
        hs, ds = self.blockers(depth, alt)
        trig = self.p['trigger_dist']

        # A level can only be called free if its slab is INSIDE the camera's vertical field of view at the
        # distance of the nearest obstacle; outside the view it is unknown, and unknown is never free.
        reach = (float(ds.min()) * self.tan_v) if ds.size else math.inf

        d_home = self.level_block_dist(hs, ds, self.home)
        home_blocked = d_home < trig
        if home_blocked:
            self.hold_active = True
            self.hold_travel = d_home + self.p['release_margin']      # refreshed while it is still seen
        elif self.hold_active:
            d_guard = self._guard_dist(hs, ds)
            if d_guard < self.p['look_ahead']:
                # something is still in the band the drone would climb / dive back through (e.g. the underside of a
                # long roof, which sits between the lowered slab and the cruise slab): never release while it is seen
                self.hold_travel = max(self.hold_travel, d_guard + self.p['release_margin'])
            else:
                self.hold_travel -= v_fwd * dt
                if self.hold_travel <= 0.0:
                    self.hold_active = False

        if not self.hold_active:
            self.target, self.status = self.home, 'cruise'
            return self.target

        d_tgt = self.level_block_dist(hs, ds, self.target)
        if d_tgt >= trig and self.target != self.home:
            self.status = 'manoeuvre: holding %.2f m' % self.target
            return self.target
        if self.target == self.home and d_home < self.p['min_start_dist']:
            # a blocker this close was not handled earlier: at this range the view of its top/bottom edge is cut off, so a
            # 'free' level is an illusion (spurious dips beside flush full-height blocks). Leave it to the lateral logic.
            self.status = 'no manoeuvre (blocker too close %.1f m): lateral logic only' % d_home
            return self.target
        # choose the free level closest to home
        best, best_cost = None, math.inf
        for a in self.levels:
            if abs(a - self.home) >= 0.15 and abs(a - alt) + self.p['half_h'] <= reach and math.isinf(self.level_block_dist(hs, ds, a)):
                cost = abs(a - self.home) + 0.01 * abs(a - alt)
                if cost < best_cost:
                    best, best_cost = float(a), cost
        if best is None:
            self.status = 'no free altitude (blocked at %.1f m): lateral logic only' % d_home
            self.target = self.home                       # never keep a half-chosen dip when nothing is really free
            return self.target
        self.target = best
        self.status = 'manoeuvre: %s to %.2f m' % ('over' if best > self.home else 'under', best)
        return best

    # ---------------------------------------------------------------- mask for the lateral logic
    def mask(self, depth, alt, far):
        """Copy of `depth` where pixels wholly outside the body slab become `far` (not an obstacle)."""
        valid, d, h, _ = self._heights(depth, alt)
        lo = alt - self.p['half_h'] - self.p['mask_margin']
        hi = alt + self.p['half_h'] + self.p['mask_margin']
        out_of_slab = valid & ((h < lo) | (h > hi))
        if not out_of_slab.any():
            return depth
        out = depth.copy()
        out[out_of_slab] = far
        return out


class CorridorFlight(Node):

    # ------------------------------------------------------------------
    # INIT
    # ------------------------------------------------------------------
    def __init__(self):
        super().__init__('corridor_flight')
        self._declare_parameters()
        self._load_parameters()
        self._create_interfaces()
        self._init_state()
        self._startup_log()

    def _declare_parameters(self):
        defaults = {
            # --- safety / topics ---
            'flight_enabled': False,
            'depth_topic': '/camera/depth/image_raw',   # Orbbec ROS 2 driver default; override if different
            'position_topic': '/fmu/out/vehicle_local_position_v1',
            'status_topic': '/fmu/out/vehicle_status_v1',
            # --- timing ---
            'control_rate_hz': 20.0,
            'depth_timeout': 0.5,
            'position_timeout': 0.5,
            'preflight_cycles': 20,
            # --- altitude ---
            'takeoff_altitude': 2.0,       # cruise height above the floor, m (rules: 8 ft = 2.44 m or lower)
            'altitude_tolerance': 0.3,
            'kp_altitude': 1.0,
            'max_climb_speed': 1.0,
            'max_descend_speed': 0.5,
            # --- camera / perception ---
            'hfov_deg': 86.0,              # MUST match the camera (Gemini 330 = 86 deg)
            'n_sectors': 31,               # keep ODD (true centre sector)
            'band_top': 0.30,              # vertical band covers the drone's body height
            'band_bottom': 0.70,
            # [VERT] vertical (over/under) manoeuvre layer. Off by default = behaviour unchanged.
            'vertical_enabled': True,
            'vert_mask': True,             # hide pixels wholly above/below the body slab from the lateral logic
            'vert_ground_offset': 0.0,     # AGL = -local_ned_z + this  (sim: 0.48, real ArduPilot: 0.0)
            'vert_alt_min': 0.6,           # lowest centre height, m AGL
            'vert_alt_max': 2.1,           # highest centre height, m AGL (body top must stay under the walls)
            'vert_half_h': 0.30,           # slab half height = body half height + margin
            'vert_path_halfwidth': 0.45,   # lateral half width of the path that is checked
            'vert_look_ahead': 4.0,        # only obstacles closer than this are considered
            'vert_trigger_dist': 3.5,      # start a manoeuvre when the cruise slab is blocked closer than this
            'vert_release_margin': 1.0,    # distance flown past the last sighting before climbing back; 0.5 climbed into an overhead beam at 0.4 m/s
            'pixel_stride': 2,
            'min_depth': 0.2,
            'max_depth': 15.0,
            'percentile': 5.0,             # low percentile catches thin obstacles
            'close_fraction': 0.10,
            'max_unknown_fraction': 0.30,  # NaN-heavy sector = blocked (dark surfaces)
            'zero_is_invalid': True,       # [REV2] kept for compatibility; zero is ALWAYS invalid now
            'sky_free': True,              # [SKY] +inf above the horizon (open sky) counts as free space
            'sky_horizon_frac': 0.50,      # [SKY] image row of the horizon (fraction of image height)
            'sky_margin_frac': 0.03,       # [SKY] rows this close to the horizon are never sky
            'partial_unknown_fraction': 0.10,  # [REV2] sector with more unknown than this is capped at unknown_range
            'nodata_width': 0.2,           # assumed obstacle width for no-data (dark) blobs, m
            'nodata_max_blob_deg': 30.0,   # wider no-data blobs = sky/untextured wall -> unknown
            'nodata_min_px': 4,            # ignore narrower no-data columns (speckle)
            # --- [REV] depth confidence / unknown space ---
            'min_finite_fraction': 0.10,   # <10 % real returns in the band -> sensor not trusted -> HOLD
                                           # (+inf / NaN / 0 / negative are UNKNOWN, never free space)
            'min_known_sectors_frac': 0.5, # fewer known sectors than this -> HOLD
            'good_known_sectors_frac': 0.8,# fewer than this -> degraded (slow) mode
            'degraded_speed_factor': 0.5,  # speed multiplier in degraded mode
            'unknown_ahead_deg': 20.0,     # unknown sector within +-this of ahead -> degraded mode
            'depth_resume_frames': 3,      # consecutive good frames before moving again after a dropout
            # --- avoidance ---
            'obstacle_distance': 2.0,      # radial range below which a sector is blocked
            'hysteresis': 0.15,
            'release_frames': 2,           # frames a heading must stay clear before release
            'creep_distance': 1.3,         # min free path to creep along when nothing is fully free
            'stop_distance': 1.0,          # centre closer than this -> lateral escape
            'safety_radius': 0.35,          # real drone motor+guard radius 0.336 m
            'unknown_range': 1.5,          # assumed range of a no-data sector
            'memory_enabled': True,        # remember obstacles that left the camera FOV
            'memory_time': 20.0,
            'memory_range': 3.5,           # only remember points closer than this
            'memory_use_range': 2.5,       # only plan around remembered points closer than this
            'memory_cell': 0.15,
            'memory_max_points': 800,
            'memory_keep_near': 1.2,       # [REV] remembered points this close to the drone do not time out
            'memory_clear_margin': 0.5,    # [REV] in-view memory point cleared if the sector now sees this much further
            'body_half_length': 0.3,
            'side_clearance': 0.75,        # lateral room required before a sidestep
            'goal_bearing_deg': 0.0,
            'w_goal': 1.0,
            'w_smooth': 0.4,
            'w_clear': 0.3,
            'center_tolerance_deg': 5.0,
            'escape_min_angle_deg': 15.0,
            'pass_gap': 0.85,               # min free width beside a blocker to pass it (m)
            'pass_gap_hyst': 0.05,
            'cross_max_deg': 35.0,         # [REV4] max bearing allowed to cross in front of a blocker
            'guard_gain': 1.5,             # [REV4] lateral push (m/s per m of intrusion); 0 = off
            'guard_dist': 0.55,            # [REV4] start pushing when anything is this close beside the body
            'guard_max': 0.25,             # [REV4] cap on the push (m/s)
            'guard_ahead': 0.6,            # [REV4] only points up to this far ahead count as 'beside'
            'gap_lookahead': 3.2,          # [REV4] start positioning for a gap this far ahead (m)
            'pre_angle_deg': 15.0,         # [REV4] bearing bias while positioning early
            'cross_commit_deg': 15.0,      # [REV4] extra bearing tolerated once a side is committed
            'gap_commit_time': 4.0,        # [REV4] how long a gap-aware side stays committed (s)
            'cluster_depth': 0.8,          # [REV4] points this close behind the nearest = same obstacle
            'w_side_stick': 1.0,           # [REV2] cost penalty for the side opposite the committed avoidance side
            # --- [REV] dynamic / sudden obstacles ---
            'escape_speed': 0.25,          # lateral escape speed when the blocker is very close
            'emergency_distance': 0.6,     # centre free path below this: full escape speed / urgent recovery
            # --- latch (keep sidestepping after obstacle leaves the FOV) ---
            'latch_time': 6.0,             # max duration of the latch
            'latch_distance': 1.2,         # forward travel kept sidestepping after the obstacle leaves the FOV
            'latch_angle_deg': 15.0,
            # --- wall centering ---
            'center_kp': 0.3,
            'center_max_speed': 0.15,
            'center_min_angle_deg': 25.0,
            'center_valid_lat': 4.0,       # wall candidates must be closer than this (lateral)
            # [REV] wall-vs-obstacle heuristic and centering shaping
            'wall_min_angle_deg': 10.0,    # sectors closer to the axis than this are never used as wall
            'wall_max_range': 6.0,         # radial range limit for wall candidates
            'wall_lat_tol': 0.30,          # lateral spread allowed inside one wall run (m)
            'wall_min_sectors': 4,         # minimum run length
            'wall_min_span': 0.8,          # run must extend >= this far along the heading (rejects box faces)
            'wall_confirm_frames': 3,      # frames before a wall estimate is trusted
            'wall_lost_time': 1.0,         # s without a wall run before the estimate is dropped
            'wall_filter_alpha': 0.3,      # low-pass on lateral wall distance
            'wall_max_yaw_err_deg': 8.0,   # [REV2] yaw further than this from the corridor axis -> no wall estimate
            'wall_jump': 0.6,              # [REV2] sudden lateral wall jump (m) = local obstacle, not wall
            'center_filter_alpha': 0.15,   # [REV2] low-pass on the centering velocity
            'center_side_extra_lat': 0.25, # [REV2] extra lateral margin to remembered points (used by the stall-recovery escape check)
            'center_probe_dist': 1.0,      # [REV4] free path needed on the 20 deg centering probe. NOT eff_block (2.0): the far wall alone limits a
                                           #   20 deg path to ~1.5-2.0 m in a 1.65 m corridor, which vetoed centering for every offset < ~0.22 m
            'centering_extra_lat': -0.20,  # [REV4] same, but for wall CENTERING only: added to side_clearance (0.75) -> vetoed only by a remembered
                                           #   point closer than 0.55 m sideways. With +0.25 (1.0 m) the far WALL (0.825+offset, remembered) vetoed
                                           #   centering for any offset < ~0.2 m in a 1.65 m corridor: the drone stayed ~0.2 m off-centre after a dodge.
            'center_side_extra_fwd': 0.4,  # [REV2] extra fore/aft window for remembered points when centering
            'center_deadband': 0.15,       # on (right-left) wall distance difference, m
            'center_resume_delay': 0.5,    # s after avoidance ends before centering starts
            'center_ramp_time': 2.0,       # s to ramp centering gain 0 -> 1
            'center_max_speed_close': 0.25,# centering limit when a wall is closer than wall_warn_lat
            'wall_warn_lat': 1.0,
            'corridor_width': 1.65,         # briefing: 5-6 ft (1.5-1.8 m); wall centring needs this within tol of the real width
            'corridor_width_tol': 0.4,     # both-wall estimate must be within this of corridor_width, else no centring
            'pose_max_eph': 1.0,           # [REV3] EKF horizontal error above this (or unknown) -> memory off, slow mode
            'single_wall_centering': False,# off by default: needs SITL validation
            # --- stuck recovery ---
            'stuck_timeout': 3.0,
            'relax_step': 0.25,
            'relax_interval': 2.0,
            'min_block_distance': 1.2,     # hard floor for relaxed threshold
            'restore_step': 0.02,
            'retreat_enabled': True,       # [REV] back-off step of recovery; now only along the flown path
            'retreat_after': 8.0,
            'retreat_speed': 0.15,
            'retreat_time': 2.0,
            'retreat_cooldown': 6.0,
            # [REV] deadlock / trapped-state recovery
            'recovery_enabled': True,
            'recovery_stop_time': 1.0,
            'recovery_eval_time': 1.5,
            'recovery_max_attempts': 3,
            'trail_spacing': 0.25,
            'trail_max_points': 80,
            'trail_lateral_tol': 0.4,
            'progress_window': 6.0,        # s
            'progress_min_dist': 0.25,     # m moved inside the window, otherwise "no progress"
            'osc_window': 6.0,
            'osc_flips': 4,
            'fwd_progress_window': 10.0,   # [REV2] s
            'fwd_progress_min': 0.2,       # [REV2] m forward inside the window, otherwise "insufficient progress"
            'recovery_escape_time': 2.5,   # [REV2] max duration of the guarded lateral escape step
            'recovery_escape_speed': 0.15, # [REV2] lateral speed of that step
            # --- motion ---
            'max_speed': 0.4,
            'min_speed': 0.15,
            'max_accel': 1.0,
            'max_decel': 2.0,
            # --- heading ---
            'use_initial_heading': True,
            'target_yaw_deg': 0.0,
            # --- shutdown ---
            'land_on_exit': True,
        }
        # dynamic typing: `-p max_speed:=1` (int) is accepted for float params
        desc = ParameterDescriptor(dynamic_typing=True)
        for name, value in defaults.items():
            self.declare_parameter(name, value, desc)

    def _load_parameters(self):
        g = lambda n: self.get_parameter(n).value  # noqa: E731
        self.flight_enabled = bool(g('flight_enabled'))
        self.depth_topic = g('depth_topic')
        self.position_topic = g('position_topic')
        self.status_topic = g('status_topic')
        self.control_rate = float(g('control_rate_hz'))
        self.dt = 1.0 / self.control_rate
        self.depth_timeout = float(g('depth_timeout'))
        self.position_timeout = float(g('position_timeout'))
        self.preflight_cycles = int(g('preflight_cycles'))
        self.takeoff_alt = float(g('takeoff_altitude'))
        self._init_vertical(g)                        # [VERT]
        self.alt_tol = float(g('altitude_tolerance'))
        self.kp_alt = float(g('kp_altitude'))
        self.max_climb = float(g('max_climb_speed'))
        self.max_descend = float(g('max_descend_speed'))
        self.hfov_deg = float(g('hfov_deg'))
        self.n_sectors = int(g('n_sectors'))
        self.band_top = float(g('band_top'))
        self.band_bottom = float(g('band_bottom'))
        self.stride = max(1, int(g('pixel_stride')))
        self.min_depth = float(g('min_depth'))
        self.max_depth = float(g('max_depth'))
        self.percentile = float(g('percentile'))
        self.close_frac = float(g('close_fraction'))
        self.max_unknown_frac = float(g('max_unknown_fraction'))
        self.zero_is_invalid = bool(g('zero_is_invalid'))
        self.nodata_width = float(g('nodata_width'))
        self.nodata_max_blob = math.radians(float(g('nodata_max_blob_deg')))
        self.nodata_min_px = int(g('nodata_min_px'))
        self.min_finite_frac = float(g('min_finite_fraction'))
        self.min_known_sectors = float(g('min_known_sectors_frac'))
        self.good_known_sectors = float(g('good_known_sectors_frac'))
        self.degraded_factor = float(g('degraded_speed_factor'))
        self.unknown_ahead = math.radians(float(g('unknown_ahead_deg')))
        self.depth_resume_frames = int(g('depth_resume_frames'))
        self.block_dist = float(g('obstacle_distance'))
        self.hyst = float(g('hysteresis'))
        self.release_frames = int(g('release_frames'))
        self.creep_dist = float(g('creep_distance'))
        self.stop_dist = float(g('stop_distance'))
        self.safety_radius = float(g('safety_radius'))
        self.unknown_range = float(g('unknown_range'))
        self.mem_enabled = bool(g('memory_enabled'))
        self.mem_time = float(g('memory_time'))
        self.mem_range = float(g('memory_range'))
        self.mem_use_range = float(g('memory_use_range'))
        self.mem_cell = max(0.05, float(g('memory_cell')))
        self.mem_max = int(g('memory_max_points'))
        self.mem_keep_near = float(g('memory_keep_near'))
        self.mem_clear_margin = float(g('memory_clear_margin'))
        self.body_half = float(g('body_half_length'))
        self.side_clearance = float(g('side_clearance'))
        self.goal = math.radians(float(g('goal_bearing_deg')))
        self.w_goal = float(g('w_goal'))
        self.w_smooth = float(g('w_smooth'))
        self.w_clear = float(g('w_clear'))
        self.center_tol = math.radians(float(g('center_tolerance_deg')))
        self.escape_min_angle = math.radians(float(g('escape_min_angle_deg')))
        self.escape_speed = float(g('escape_speed'))
        self.emergency_dist = float(g('emergency_distance'))
        self.latch_time = float(g('latch_time'))
        self.latch_distance = float(g('latch_distance'))
        self.latch_angle = math.radians(float(g('latch_angle_deg')))
        self.center_kp = float(g('center_kp'))
        self.center_max = float(g('center_max_speed'))
        self.center_min_angle = math.radians(float(g('center_min_angle_deg')))
        self.center_valid_lat = float(g('center_valid_lat'))
        self.wall_min_angle = math.radians(float(g('wall_min_angle_deg')))
        self.wall_max_range = float(g('wall_max_range'))
        self.wall_lat_tol = float(g('wall_lat_tol'))
        self.wall_min_sectors = int(g('wall_min_sectors'))
        self.wall_min_span = float(g('wall_min_span'))
        self.wall_confirm = int(g('wall_confirm_frames'))
        self.wall_lost_time = float(g('wall_lost_time'))
        self.wall_alpha = float(g('wall_filter_alpha'))
        self.center_deadband = float(g('center_deadband'))
        self.center_resume_delay = float(g('center_resume_delay'))
        self.center_ramp_time = float(g('center_ramp_time'))
        self.center_max_close = float(g('center_max_speed_close'))
        self.wall_warn_lat = float(g('wall_warn_lat'))
        self.corridor_width = float(g('corridor_width'))
        self.corridor_width_tol = float(g('corridor_width_tol'))
        self.pose_max_eph = float(g('pose_max_eph'))
        self.single_wall = bool(g('single_wall_centering'))
        self.stuck_timeout = float(g('stuck_timeout'))
        self.relax_step = float(g('relax_step'))
        self.relax_interval = float(g('relax_interval'))
        self.min_block_dist = float(g('min_block_distance'))
        self.restore_step = float(g('restore_step'))
        self.retreat_enabled = bool(g('retreat_enabled'))
        self.retreat_after = float(g('retreat_after'))
        self.retreat_speed = float(g('retreat_speed'))
        self.retreat_time = float(g('retreat_time'))
        self.retreat_cooldown = float(g('retreat_cooldown'))
        self.recovery_enabled = bool(g('recovery_enabled'))
        self.recovery_stop_time = float(g('recovery_stop_time'))
        self.recovery_eval_time = float(g('recovery_eval_time'))
        self.recovery_max_attempts = int(g('recovery_max_attempts'))
        self.trail_spacing = max(0.1, float(g('trail_spacing')))
        self.trail_max = int(g('trail_max_points'))
        self.trail_lat_tol = float(g('trail_lateral_tol'))
        self.progress_window = float(g('progress_window'))
        self.progress_min = float(g('progress_min_dist'))
        self.osc_window = float(g('osc_window'))
        self.osc_flips = int(g('osc_flips'))
        self.partial_unknown_frac = float(g('partial_unknown_fraction'))
        self.sky_free = bool(g('sky_free'))
        self.sky_horizon_frac = float(g('sky_horizon_frac'))
        self.sky_margin_frac = float(g('sky_margin_frac'))
        self.w_side_stick = float(g('w_side_stick'))
        self.pass_gap = float(g('pass_gap'))
        self.pass_gap_hyst = float(g('pass_gap_hyst'))
        self.cross_max = math.radians(float(g('cross_max_deg')))
        self.cluster_depth = float(g('cluster_depth'))
        self.gap_lookahead = float(g('gap_lookahead'))
        self.guard_gain = float(g('guard_gain'))
        self.guard_dist = float(g('guard_dist'))
        self.guard_max = float(g('guard_max'))
        self.guard_ahead = float(g('guard_ahead'))
        self.pre_angle = math.radians(float(g('pre_angle_deg')))
        self.cross_commit = math.radians(float(g('cross_commit_deg')))
        self.gap_commit_time = float(g('gap_commit_time'))
        self.wall_max_yaw_err = math.radians(float(g('wall_max_yaw_err_deg')))
        self.wall_jump = float(g('wall_jump'))
        self.center_alpha = float(g('center_filter_alpha'))
        self.center_extra_lat = float(g('center_side_extra_lat'))
        self.centering_extra_lat = float(g('centering_extra_lat'))
        self.center_probe_dist = float(g('center_probe_dist'))
        self.center_extra_fwd = float(g('center_side_extra_fwd'))
        self.fwd_window = float(g('fwd_progress_window'))
        self.fwd_min = float(g('fwd_progress_min'))
        self.rec_esc_time = float(g('recovery_escape_time'))
        self.rec_esc_speed = float(g('recovery_escape_speed'))
        self.max_speed = float(g('max_speed'))
        self.min_speed = float(g('min_speed'))
        self.max_accel = float(g('max_accel'))
        self.max_decel = float(g('max_decel'))
        self.use_initial_heading = bool(g('use_initial_heading'))
        self.target_yaw = math.radians(float(g('target_yaw_deg')))
        self.land_on_exit = bool(g('land_on_exit'))
        self._validate_parameters()

    # ------------------------------------------------------------------
    # [VERT] VERTICAL MANOEUVRE LAYER (optional; see vertical_layer.py)
    # ------------------------------------------------------------------
    def _init_vertical(self, g):
        self.vert_enabled = bool(g('vertical_enabled'))
        self.alt_target = self.takeoff_alt            # altitude set-point actually flown (same units as takeoff_altitude)
        self.vert, self.vert_armed, self.vert_status = None, False, ''
        self.vert_mask = bool(g('vert_mask'))
        self.vert_z0 = float(g('vert_ground_offset'))
        self.vert_far = float(g('max_depth'))
        if not self.vert_enabled:
            return
        self.vert = VerticalPlanner(
            self.takeoff_alt + self.vert_z0, alt_min=float(g('vert_alt_min')), alt_max=float(g('vert_alt_max')),
            half_h=float(g('vert_half_h')), path_halfwidth=float(g('vert_path_halfwidth')),
            look_ahead=float(g('vert_look_ahead')), trigger_dist=float(g('vert_trigger_dist')),
            release_margin=float(g('vert_release_margin')), hfov_deg=float(g('hfov_deg')))

    def _vertical_step(self, depth):
        """Runs on every depth frame. Sets self.alt_target; returns the depth image the lateral logic should use."""
        if not self.vert_enabled or self.vert is None:
            return depth
        try:
            now = self.now_s()
            if not self._pose_ok(now):
                return depth
            if not self.vert_armed:                    # only after the normal take-off has reached cruise
                if abs(self.current_z + self.takeoff_alt) < 2.0 * self.alt_tol:
                    self.vert_armed = True
                    self.get_logger().info('[VERT] vertical layer armed')
                return depth
            alt = -self.current_z + self.vert_z0
            tgt = self.vert.update(depth, alt, math.hypot(self.cur_vn, self.cur_ve), now)
            self.alt_target = tgt - self.vert_z0
            if self.vert.status != self.vert_status:
                self.vert_status = self.vert.status
                self.get_logger().info(f'[VERT] {self.vert_status} (alt {alt:.2f} m AGL)')
            return self.vert.mask(depth, alt, self.vert_far) if self.vert_mask else depth
        except Exception as exc:                       # never let the optional layer take the node down
            self.get_logger().error(f'[VERT] layer disabled after error: {exc!r}')
            self.vert_enabled = False
            self.alt_target = self.takeoff_alt
            return depth

    def _validate_parameters(self):
        """Correct impossible combinations instead of flying with them."""
        log = self.get_logger()
        before = (self.n_sectors, self.stop_dist, self.block_dist, self.min_block_dist,
                  self.creep_dist, self.max_speed, self.min_speed, self.band_top,
                  self.band_bottom, self.hfov_deg, self.release_frames)
        self.n_sectors = max(3, self.n_sectors)
        if self.n_sectors % 2 == 0:
            self.n_sectors += 1                       # need a true centre sector
        self.hfov_deg = clamp(self.hfov_deg, 10.0, 170.0)
        if not 0.0 <= self.band_top < self.band_bottom <= 1.0:
            self.band_top, self.band_bottom = 0.30, 0.70
        self.stop_dist = max(self.stop_dist, 0.3)
        self.block_dist = max(self.block_dist, self.stop_dist + 0.3)
        self.min_block_dist = clamp(self.min_block_dist, self.stop_dist + 0.1, self.block_dist)
        self.creep_dist = max(self.creep_dist, self.stop_dist + 0.1)
        self.max_speed = max(self.max_speed, 0.05)
        self.min_speed = clamp(self.min_speed, 0.05, self.max_speed)
        self.release_frames = max(1, self.release_frames)
        self.hyst = max(0.0, self.hyst)
        self.percentile = clamp(self.percentile, 0.0, 50.0)
        after = (self.n_sectors, self.stop_dist, self.block_dist, self.min_block_dist,
                 self.creep_dist, self.max_speed, self.min_speed, self.band_top,
                 self.band_bottom, self.hfov_deg, self.release_frames)
        if before != after:
            log.warn(f'Parameters corrected for safety: {before} -> {after}')

        # [REV] validation of the new parameters
        b2 = (self.escape_speed, self.emergency_dist, self.degraded_factor,
              self.min_known_sectors, self.good_known_sectors, self.min_finite_frac,
              self.depth_resume_frames, self.wall_min_sectors, self.wall_alpha,
              self.center_max_close, self.recovery_max_attempts, self.osc_flips)
        self.degraded_factor = clamp(self.degraded_factor, 0.1, 1.0)
        self.min_known_sectors = clamp(self.min_known_sectors, 0.0, 1.0)
        self.good_known_sectors = clamp(self.good_known_sectors, self.min_known_sectors, 1.0)
        self.min_finite_frac = clamp(self.min_finite_frac, 0.0, 1.0)
        self.emergency_dist = clamp(self.emergency_dist, 0.2, self.stop_dist - 0.1)
        self.escape_speed = clamp(self.escape_speed, self.min_speed, self.max_speed)
        self.depth_resume_frames = max(1, self.depth_resume_frames)
        self.wall_min_sectors = max(3, self.wall_min_sectors)
        self.wall_alpha = clamp(self.wall_alpha, 0.05, 1.0)
        self.center_max_close = clamp(self.center_max_close, self.center_max, self.max_speed)
        self.recovery_max_attempts = max(1, self.recovery_max_attempts)
        self.osc_flips = max(2, self.osc_flips)
        self.partial_unknown_frac = clamp(self.partial_unknown_frac, 0.0, self.max_unknown_frac)
        self.center_alpha = clamp(self.center_alpha, 0.02, 1.0)
        self.corridor_width_tol = max(0.2, self.corridor_width_tol)
        self.pose_max_eph = max(0.1, self.pose_max_eph)
        self.wall_jump = max(0.1, self.wall_jump)
        self.fwd_window = max(self.fwd_window, self.progress_window)
        self.rec_esc_time = max(0.5, self.rec_esc_time)
        self.rec_esc_speed = clamp(self.rec_esc_speed, self.min_speed, self.escape_speed)
        a2 = (self.escape_speed, self.emergency_dist, self.degraded_factor,
              self.min_known_sectors, self.good_known_sectors, self.min_finite_frac,
              self.depth_resume_frames, self.wall_min_sectors, self.wall_alpha,
              self.center_max_close, self.recovery_max_attempts, self.osc_flips)
        if b2 != a2:
            log.warn(f'New parameters corrected for safety: {b2} -> {a2}')

    def _create_interfaces(self):
        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        # BEST_EFFORT subscriber matches both reliable and best-effort publishers.
        depth_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(Image, self.depth_topic, self.depth_callback, depth_qos)
        self.create_subscription(
            VehicleLocalPosition, self.position_topic, self.position_callback, px4_qos)
        self.create_subscription(
            VehicleStatus, self.status_topic, self.status_callback, px4_qos)

        self.decision_pub = self.create_publisher(String, '/depth_navigation/decision', 10)
        self.clearance_pub = self.create_publisher(
            Float32MultiArray, '/depth_navigation/sector_clearance', 10)
        # [REV] additive, optional diagnostics (confidence, wall/obstacle classes, recovery)
        self.diag_pub = self.create_publisher(String, '/depth_navigation/diagnostics', 10)
        self.offboard_mode_pub = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', px4_qos)
        self.trajectory_pub = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', px4_qos)
        self.vehicle_command_pub = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', px4_qos)

        self.timer = self.create_timer(self.dt, self.control_loop)

    def _init_state(self):
        n = self.n_sectors
        if n % 2 == 0:
            self.get_logger().warn('n_sectors is even: there is no true centre sector.')

        self.state = S_WAIT
        self.cycle = 0
        self.preflight_count = 0

        self.depth_received = False
        self.position_received = False
        self.status_received = False
        self.last_depth_time = -1e9
        self.last_position_time = -1e9
        self.pose_valid = True
        self.current_z = 0.0
        self.pos_x = 0.0
        self.pos_y = 0.0
        self.mem = {}
        self.heading = 0.0
        self.heading_valid = False
        self.pose_degraded = False     # [REV3] EKF position quality poor / unknown
        self.yaw_locked = False        # [REV2] hold_yaw is NOT valid until established
        self.armed = False
        self.offboard = False

        self._geom = None
        self.angles = np.zeros(n)
        self.col_scale = None
        self.sector_width = math.radians(self.hfov_deg) / n
        self.clearance = np.full(n, self.max_depth)
        self.blocked = np.zeros(n, dtype=bool)
        self.clear_count = np.zeros(n, dtype=int)
        self.nav_angle = None
        self.nav_speed = 0.0
        self.nav_vy = 0.0
        self.prev_angle = self.goal
        self.last_decision = ''

        # latch / stuck recovery
        self.latch_dir = 0
        self.latch_until = -1e9
        self.latch_dist = 0.0
        self.escape_dir = 0
        self.eff_block = self.block_dist
        self.hold_since = None
        self.last_relax = -1e9
        self.retreat_until = -1e9
        self.cmd_body = (0.0, 0.0)

        # [REV] perception confidence
        self.percep_factor = 0.0
        self.percep_hold = True
        self.percep_reason = 'no depth yet'
        self.good_frames = 0
        # [REV] wall / obstacle classification
        self.sector_class = np.zeros(n, dtype=np.int8)
        self.wall_conf = {-1: 0, 1: 0}
        self.wall_lat = {-1: None, 1: None}
        self.wall_seen = {-1: -1e9, 1: -1e9}
        self.width_est = None
        self.last_avoid_time = -1e9
        self.escaping = False
        self.center_fd = self.max_depth
        self._cgate = '-'
        self._cerr = 0.0
        self._cramp = 0.0
        self._fd_last = None
        self.vy_filt = 0.0
        # [REV] recovery
        self.rec_state = ''
        self.rec_t0 = 0.0
        self.rec_urgent = False
        self.rec_attempts = 0
        self.rec_pos = None
        self.rec_cooldown_until = -1e9
        self.rec_side = 0
        self.rec_escaped = False
        self.trail = []
        self.trail_last = None
        self.pos_hist = []
        self.side_flips = []
        self.last_side = 0
        self.gap_side = 0
        self.gap_until = -1e9

        self.hold_yaw = 0.0
        self.cur_vn = 0.0
        self.cur_ve = 0.0

    def _startup_log(self):
        log = self.get_logger()
        log.info('==========================================')
        log.info('   X500 DEPTH CAMERA NAVIGATION')
        log.info('==========================================')
        log.info(f'Depth: {self.depth_topic} | Sectors: {self.n_sectors} '
                 f'| HFOV: {self.hfov_deg:.0f} deg')
        log.info(f'Speed {self.min_speed:.2f}-{self.max_speed:.2f} m/s | block {self.block_dist:.2f} m '
                 f'| stop {self.stop_dist:.2f} m | emergency {self.emergency_dist:.2f} m '
                 f'| safety radius {self.safety_radius:.2f} m | corridor {self.corridor_width:.1f} m')
        if self.max_speed * 2.0 > self.stop_dist:
            log.warn('max_speed is high relative to stop_distance: re-check braking margin.')
        if self.flight_enabled:
            log.warn('FLIGHT ENABLED: node will arm, take off and fly.')
        else:
            log.warn('SAFE TEST MODE: FLIGHT DISABLED (perception + decisions only). '
                     'Enable with -p flight_enabled:=true')

    # ------------------------------------------------------------------
    # HELPERS
    # ------------------------------------------------------------------
    def now_s(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def timestamp(self):
        return int(self.get_clock().now().nanoseconds / 1000)

    def _lock_yaw(self, force=False):
        """[REV2] Establish hold_yaw (the corridor axis) from the real heading."""
        if self.yaw_locked and not force:
            return
        if self.use_initial_heading or not self.flight_enabled:
            if not self.heading_valid:
                return
            ref = self.heading
        else:
            ref = self.target_yaw
        self.hold_yaw = ref
        self.yaw_locked = True

    def _yaw_ready(self):
        """[REV2] Body<->world transforms (memory, trail) are only meaningful with a real yaw."""
        return self.yaw_locked and (not self.flight_enabled or self.state == S_NAVIGATE)

    def _yaw_consistent(self):
        """[REV2] Is the vehicle still aligned with the corridor axis (needed for wall geometry)?"""
        if not self.yaw_locked or not self.heading_valid:
            return False
        err = math.atan2(math.sin(self.heading - self.hold_yaw),
                         math.cos(self.heading - self.hold_yaw))
        return abs(err) <= self.wall_max_yaw_err

    # ------------------------------------------------------------------
    # CALLBACKS
    # ------------------------------------------------------------------
    def position_callback(self, msg):
        self.position_received = True
        self.last_position_time = self.now_s()
        self.current_z = float(msg.z)
        x, y = float(getattr(msg, 'x', 0.0)), float(getattr(msg, 'y', 0.0))
        if math.isfinite(x) and math.isfinite(y):
            self.pos_x, self.pos_y = x, y
        self.pose_valid = bool(getattr(msg, 'xy_valid', True)
                               and getattr(msg, 'z_valid', True))
        eph = float(getattr(msg, 'eph', 0.0))
        self.pose_degraded = bool((not math.isfinite(eph)) or eph > self.pose_max_eph
                                  or not getattr(msg, 'v_xy_valid', True))
        heading = float(getattr(msg, 'heading', 0.0))
        if math.isfinite(heading):
            self.heading = heading
            self.heading_valid = bool(getattr(msg, 'heading_good_for_control', True))
        else:
            self.heading_valid = False

    def status_callback(self, msg):
        self.status_received = True
        self.armed = msg.arming_state == VehicleStatus.ARMING_STATE_ARMED
        self.offboard = msg.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD

    def depth_callback(self, msg):
        try:
            self._process_depth(msg)
        except Exception as e:  # noqa: BLE001
            # Never keep flying on a stale decision if processing breaks.
            self._perception_fail(f'processing error: {e}')
            self.get_logger().error(f'Depth processing failed: {e}',
                                    throttle_duration_sec=2.0)

    def _perception_fail(self, reason):
        """[REV] Unusable frame: stop, and require fresh good frames before moving again."""
        self.nav_angle, self.nav_speed, self.nav_vy = None, 0.0, 0.0
        self.percep_hold, self.percep_factor, self.percep_reason = True, 0.0, reason
        self.good_frames = 0
        self.hold_since = None

    def _assess_confidence(self, finite_frac, known_frac, clearance):
        """
        [REV] Returns (speed_factor, hold, reason). Unknown space is never treated as free:
        no / few real returns or mostly unknown sectors -> HOLD; partly unknown -> slow.
        """
        if finite_frac < self.min_finite_frac:
            return 0.0, True, f'no valid returns ({finite_frac:.0%} finite)'
        if known_frac < self.min_known_sectors:
            return 0.0, True, f'mostly unknown ({known_frac:.0%} sectors known)'
        factor, reason = 1.0, 'ok'
        if known_frac < self.good_known_sectors:
            factor, reason = self.degraded_factor, f'partly unknown ({known_frac:.0%} known)'
        ahead = np.abs(self.angles) <= self.unknown_ahead
        if np.any(clearance[ahead] < 0.0):
            factor = min(factor, self.degraded_factor)
            reason = 'unknown sector ahead'
        return factor, False, reason

    def _process_depth(self, msg):
        depth = self.decode_depth(msg)
        if depth is None:
            self._perception_fail('depth frame unusable')
            return
        depth = self._vertical_step(depth)            # [VERT]

        height, width = depth.shape
        _, edges, _ = self._update_geometry(width)

        y1 = int(height * self.band_top)
        y2 = max(y1 + 1, int(height * self.band_bottom))
        band = depth[y1:y2:self.stride, :]
        # [REV2] NaN, +inf, -inf, zero and negative depth are all UNKNOWN (NaN). +inf is
        # NOT free space and is never converted to max_depth.
        valid = np.isfinite(band) & (band > 0.0)
        # [SKY] open sky returns +inf. It is free space only where the same column also has a real
        # return at/below the horizon, so a dark/no-data surface (+inf in every row) stays unknown.
        sky = np.zeros(band.shape, dtype=bool)
        if self.sky_free:
            cut = int(height * self.sky_horizon_frac) - int(height * self.sky_margin_frac)
            above = (y1 + np.arange(band.shape[0]) * self.stride) < cut
            has_ground = (valid & ~above[:, None]).any(axis=0)
            sky = np.isposinf(band) & above[:, None] & has_ground[None, :]
        finite_frac = float(np.mean(valid | sky))
        band = np.where(valid, band, np.nan).astype(np.float32)
        # Sensor reports z-depth; convert to radial (horizontal-plane) range so
        # side walls / edge pixels are not under-estimated.
        band = band * self.col_scale[None, :]

        n = self.n_sectors
        clearance = np.empty(n)
        for i in range(n):
            clearance[i] = self.sector_clearance(band[:, edges[i]:edges[i + 1]:self.stride],
                                                 sky[:, edges[i]:edges[i + 1]:self.stride])
        band_nb = np.where(sky, np.float32(self.max_depth), band)   # [SKY] sky is not a no-data blob
        clearance = self._apply_nodata_blobs(band_nb, edges, clearance)
        # [REV3] Median-filter clearance across the last few real frames. Absorbs a single
        # corrupted/stale render (seen under software-rendered Gazebo) without masking a
        # genuinely fast-closing obstacle, since a true approach stays low across frames too.
        if not hasattr(self, '_clearance_hist'):
            self._clearance_hist = []
        self._clearance_hist.append(clearance)
        if len(self._clearance_hist) > 3:
            self._clearance_hist.pop(0)
        clearance = np.median(np.stack(self._clearance_hist, axis=0), axis=0)
        self.clearance = clearance

        now = self.now_s()
        self.last_depth_time = now
        self.depth_received = True
        known_frac = float(np.mean(clearance >= 0.0))
        factor, hold, reason = self._assess_confidence(finite_frac, known_frac, clearance)
        if hold and not self.percep_hold:
            self.get_logger().warn(f'Depth unreliable -> HOLD ({reason})')
        elif not hold and self.percep_hold:
            self.get_logger().info('Valid depth restored')
        if self.pose_degraded and not hold:             # [REV3] memory/latch less trustworthy
            factor = min(factor, self.degraded_factor)
            reason = 'pose degraded' if reason == 'ok' else reason + ' + pose degraded'
        self.percep_factor, self.percep_hold, self.percep_reason = factor, hold, reason

        out = Float32MultiArray()
        out.data = [float(c) for c in clearance]
        self.clearance_pub.publish(out)

        if hold:                                   # [REV] never plan on unreliable data
            self.nav_angle, self.nav_speed, self.nav_vy = None, 0.0, 0.0
            self.good_frames = 0
            self.hold_since = None
            return
        self.good_frames += 1

        self._update_memory(clearance, now)
        self._update_walls(clearance, now)         # [REV] wall-vs-obstacle classification
        angle, speed, vy = self.choose_direction(clearance, now)
        if not self.escaping:                      # [REV] degraded depth -> conservative speed
            speed *= factor
            vy *= factor
        if angle is not None and speed > 0.0:
            vy += self._side_guard(clearance)      # [REV4] reactive protection against drift
        self._update_stuck(angle, speed, now)
        self.nav_angle, self.nav_speed, self.nav_vy = angle, speed, vy

    # ------------------------------------------------------------------
    # PERCEPTION
    # ------------------------------------------------------------------
    def decode_depth(self, msg):
        if msg.encoding == '32FC1':
            dtype, scale = np.float32, 1.0
        elif msg.encoding in ('16UC1', 'mono16'):
            dtype, scale = np.uint16, 0.001
        else:
            self.get_logger().error(
                f'Unsupported depth encoding: {msg.encoding}', throttle_duration_sec=5.0)
            return None
        itemsize = np.dtype(dtype).itemsize
        row_len = (msg.step // itemsize) if msg.step else msg.width
        raw = np.frombuffer(msg.data, dtype=dtype)
        if raw.size < msg.height * row_len:
            raise ValueError(f'truncated depth frame {raw.size}/{msg.height * row_len}')
        raw = raw[:msg.height * row_len]
        raw = raw.reshape((msg.height, row_len))[:, :msg.width]
        return raw.astype(np.float32) * scale

    def _update_geometry(self, width):
        if self._geom is not None and self._geom[0] == width:
            return self._geom
        n = self.n_sectors
        if width < 4 * n:
            raise ValueError(f'image width {width} too small for {n} sectors')
        edges = np.linspace(0, width, n + 1).astype(int)
        centers = (edges[:-1] + edges[1:]) / 2.0
        fx = (width / 2.0) / math.tan(math.radians(self.hfov_deg) / 2.0)
        self.angles = np.arctan((centers - width / 2.0) / fx)   # + = right
        cols = (np.arange(width) + 0.5) - width / 2.0
        self.col_scale = (1.0 / np.cos(np.arctan(cols / fx))).astype(np.float32)
        self.sector_width = float(np.mean(np.diff(self.angles)))
        self._geom = (width, edges, self.angles)
        return self._geom

    def _apply_nodata_blobs(self, band, edges, clearance):
        """
        Columns with no depth at all (black / matte / out-of-range objects) carry no range.
        Estimate it conservatively from apparent size: a blob of angular width alpha is
        assumed to be an obstacle `nodata_width` wide, so range <= width / (2 tan(alpha/2)).
        This avoids phantom obstacles for far blobs and under-estimating near ones.
        Very wide blobs (sky, untextured wall) stay 'unknown' (-1): blocked, never creep-able.
        """
        width = band.shape[1]
        col_nodata = np.isnan(band).mean(axis=0) > 0.5
        if not col_nodata.any():
            return clearance
        fx = (width / 2.0) / math.tan(math.radians(self.hfov_deg) / 2.0)
        edge = np.diff(np.concatenate(([0], col_nodata.astype(np.int8), [0])))
        for c0, c1 in zip(np.flatnonzero(edge == 1), np.flatnonzero(edge == -1)):
            if c1 - c0 < self.nodata_min_px:
                continue
            alpha = (math.atan((c1 - width / 2.0) / fx) - math.atan((c0 - width / 2.0) / fx))
            wide = alpha > self.nodata_max_blob
            r_est = max(0.3, self.nodata_width / (2.0 * math.tan(alpha / 2.0)))
            for i in range(len(clearance)):
                if min(edges[i + 1], c1) - max(edges[i], c0) < 2:
                    continue                          # blob does not touch this sector
                if wide:
                    clearance[i] = -1.0
                elif clearance[i] < 0.0:
                    clearance[i] = r_est
                else:
                    clearance[i] = min(clearance[i], r_est)
        return clearance

    def sector_clearance(self, region, sky=None):
        """
        [REV2] NaN / +inf / -inf / 0 / negative -> UNKNOWN (never free). A sector with more
        unknown pixels than max_unknown_frac -> -1.0 (no usable data). A partly unknown
        sector is capped at unknown_range so unknown pixels are never read as far clearance.
        Returns metres, 0.0 = obstacle at the lens, -1.0 = no usable data.
        """
        if region.size == 0:
            return 0.0
        if sky is not None and sky.shape == region.shape and sky.any():
            region = region[~sky]            # [SKY] open sky is free space, not a reading
            if region.size == 0:
                return float(self.max_depth)
        region = np.where(np.isfinite(region) & (region > 0.0), region, np.nan)
        nan_mask = np.isnan(region)
        unk_frac = float(nan_mask.mean())
        if unk_frac > self.max_unknown_frac:
            return -1.0                      # mostly no data (dark surface / dropout / inf)
        known = region[~nan_mask]
        if known.size == 0:
            return -1.0
        too_close = known <= self.min_depth
        if too_close.mean() > self.close_frac:
            return 0.0
        usable = known[~too_close]
        if usable.size == 0:
            return 0.0
        usable = np.minimum(usable, self.max_depth)
        c = float(np.percentile(usable, self.percentile))
        if unk_frac > self.partial_unknown_frac:
            c = min(c, self.unknown_range)   # conservative: unknown part may hide an obstacle
        return c

    # ------------------------------------------------------------------
    # DECISION
    # ------------------------------------------------------------------
    # ------------------------------------------------------------------
    # SHORT-TERM OBSTACLE MEMORY (obstacles beside the drone leave the camera FOV)
    # ------------------------------------------------------------------
    def _pose_ok(self, now):
        return (now - self.last_position_time) < self.position_timeout and self.pose_valid

    def _update_memory(self, clearance, now):
        if (not self.mem_enabled or not self._pose_ok(now) or not self._yaw_ready()
                or self.pose_degraded):
            return                                  # [REV3] no new points from a poor position fix                                  # [REV2] never remember with an unestablished yaw
        self._clear_ghosts(clearance, now)
        c, sn = math.cos(self.hold_yaw), math.sin(self.hold_yaw)
        for i in np.flatnonzero((clearance > self.min_depth) & (clearance < self.mem_range)):
            fwd = clearance[i] * math.cos(self.angles[i])
            rgt = clearance[i] * math.sin(self.angles[i])
            wx = self.pos_x + fwd * c - rgt * sn
            wy = self.pos_y + fwd * sn + rgt * c
            self.mem[(int(round(wx / self.mem_cell)), int(round(wy / self.mem_cell)))] = (wx, wy, now)
        self._prune_memory(now)

    def _clear_ghosts(self, clearance, now):
        """
        [REV] A remembered point that is IN VIEW while its sector now reports clearly more
        free range was a moving obstacle (or noise): drop it. Unknown (-1) never clears.
        """
        if not self.mem:
            return
        keys = list(self.mem.keys())
        pts = np.array([self.mem[k] for k in keys])
        c, sn = math.cos(self.hold_yaw), math.sin(self.hold_yaw)
        dx, dy = pts[:, 0] - self.pos_x, pts[:, 1] - self.pos_y
        f, y = dx * c + dy * sn, -dx * sn + dy * c
        theta, r = np.arctan2(y, f), np.hypot(f, y)
        inview = (f > 0.3) & (np.abs(theta) < math.radians(self.hfov_deg / 2.0 - 2.0))
        if not inview.any():
            return
        sec = np.argmin(np.abs(self.angles[None, :] - theta[:, None]), axis=1)
        ghost = (inview & (clearance[sec] > r + self.mem_clear_margin)
                 & ((now - pts[:, 2]) > 0.5))
        for i in np.flatnonzero(ghost):
            self.mem.pop(keys[i], None)

    def _prune_memory(self, now):
        """
        [REV] Time/range pruning on every frame (the original pruned only above 50 points,
        so small memories never expired). Points close to the drone are kept longer so a
        long HOLD beside an obstacle cannot forget it.
        """
        lim = self.mem_range + 1.0
        near_t = 3.0 * self.mem_time
        keep = {}
        for k, v in self.mem.items():
            age = now - v[2]
            d = math.hypot(v[0] - self.pos_x, v[1] - self.pos_y)
            if ((d < self.mem_keep_near and age <= near_t)
                    or (age <= self.mem_time and abs(v[0] - self.pos_x) < lim
                        and abs(v[1] - self.pos_y) < lim)):
                keep[k] = v
        self.mem = keep
        if len(self.mem) > self.mem_max:
            kept = sorted(self.mem.items(), key=lambda kv: kv[1][2])[-self.mem_max:]
            self.mem = dict(kept)

    def _memory_body(self):
        """Remembered points in the body frame: (forward, right) arrays."""
        if not self.mem_enabled or not self.mem:
            return np.zeros(0), np.zeros(0)
        pts = np.array(list(self.mem.values()))
        dx, dy = pts[:, 0] - self.pos_x, pts[:, 1] - self.pos_y
        c, sn = math.cos(self.hold_yaw), math.sin(self.hold_yaw)
        return dx * c + dy * sn, -dx * sn + dy * c

    def _memory_out_of_view(self):
        f, y = self._memory_body()
        if f.size == 0:
            return f, y
        theta = np.arctan2(y, f)
        near = np.hypot(f, y) < self.mem_use_range
        outside = (np.abs(theta) > math.radians(self.hfov_deg / 2.0 - 2.0)) | (f < 0.3)
        keep = near & outside
        return f[keep], y[keep]

    def _obstacle_points(self, clearance):
        """Visible sector points + remembered out-of-view points, body frame."""
        r = np.where(clearance > 0.0, clearance,
                     np.where(clearance == 0.0, 0.2, self.unknown_range))
        f = r * np.cos(self.angles)
        y = r * np.sin(self.angles)
        unc = r * math.sin(self.sector_width / 2.0)
        amin = np.zeros_like(r)
        unk = clearance < 0.0
        mf, my = self._memory_out_of_view()
        if mf.size:
            f = np.concatenate((f, mf))
            y = np.concatenate((y, my))
            unc = np.concatenate((unc, np.full(mf.size, 0.1)))
            amin = np.concatenate((amin, np.full(mf.size, -self.body_half)))
            unk = np.concatenate((unk, np.zeros(mf.size, dtype=bool)))
        return f, y, unc, amin, unk

    def _blocker_lateral(self, clearance, idx):
        """Signed lateral offset (+right) of the nearest point blocking heading `idx`."""
        f, y, unc, amin, _ = self._obstacle_points(clearance)
        a = float(self.angles[idx])
        along = f * math.cos(a) + y * math.sin(a)
        lat = -f * math.sin(a) + y * math.cos(a)
        hit = (along > amin) & (np.abs(lat) - unc < self.safety_radius)
        if not hit.any():
            return None
        return float(lat[int(np.argmin(np.where(hit, along, np.inf)))])

    def _path_free_distance(self, clearance):
        """
        For every candidate heading (sector centres) return how far the drone can
        travel straight before an obstacle point comes within `safety_radius` of its
        path. Open path -> max_depth. Also returns whether the path runs into a
        no-data sector.
        """
        f, y, unc, amin, unk = self._obstacle_points(clearance)
        ca = np.cos(self.angles)[:, None]
        sa = np.sin(self.angles)[:, None]
        along = f[None, :] * ca + y[None, :] * sa
        cross = np.abs(-f[None, :] * sa + y[None, :] * ca) - unc[None, :]
        hit = (along > amin[None, :]) & (cross < self.safety_radius)
        fd = np.where(hit, np.maximum(along, 0.0), np.inf).min(axis=1)
        unk_in_path = np.any(hit & unk[None, :], axis=1)
        return np.minimum(fd, self.max_depth), unk_in_path

    def _side_free(self, clearance, sign, extra_lat=0.0, extra_fwd=0.0, fwd_window=None):
        """Is there lateral room to sidestep toward `sign` (+1 right / -1 left)?"""
        r = np.where(clearance > 0.0, clearance, np.where(clearance == 0.0, 0.2, self.unknown_range))
        f = r * np.cos(self.angles)
        y = r * np.sin(self.angles)
        mf, my = self._memory_body()
        if mf.size:
            f, y = np.concatenate((f, mf)), np.concatenate((y, my))
        # fwd_window (emergency sidestep): points AHEAD of the body front are the blocker we
        # are escaping from, not a lateral hazard of a pure sideways move.
        win = self.body_half + 0.3 + extra_fwd if fwd_window is None else fwd_window
        hazard = ((np.abs(f) <= win) & (sign * y > 0.0)
                  & (sign * y < self.side_clearance + extra_lat))
        return not bool(hazard.any())

    def _nearest(self, bearing):
        return int(np.argmin(np.abs(self.angles - bearing)))

    def choose_direction(self, clearance, now):
        """
        Returns (bearing_rad | None, speed, extra_body_vy).
        bearing 0 = ahead, negative = left, positive = right, None = HOLD.
        """
        n = len(clearance)
        fd, unk = self._path_free_distance(clearance)
        self.escaping = False
        self.center_fd = float(fd[n // 2])

        # 1) Instant block, delayed release, per candidate heading
        # [REV] a path through unknown (no-data) space is never released / never free,
        # even when stuck recovery has relaxed eff_block below unknown_range.
        can_release = (fd >= self.eff_block + self.hyst) & ~unk
        self.clear_count = np.where(can_release, self.clear_count + 1, 0)
        raw_blocked = (fd < self.eff_block) | unk
        blocked = np.where(self.blocked,
                           self.clear_count < self.release_frames,
                           raw_blocked)
        self.blocked = blocked
        self._fd_last = fd
        free = ~blocked

        caution = (fd >= self.creep_dist) & ~unk   # usable slowly; never toward no-data

        # 2) Obstacle already inside stop distance -> lateral escape only
        center = n // 2
        if float(fd[center]) < self.stop_dist:
            # Pure sidestep: only the target sector itself must be open (seen obstacles
            # ahead cannot be hit by lateral motion); no-data/too-close sectors never qualify.
            cand = (clearance >= self.creep_dist) & (np.abs(self.angles) >= self.escape_min_angle)
            if cand.any():                                  # never sidestep into a remembered obstacle/wall
                esc_win = self.body_half + 0.1
                side_ok = {1: self._side_free(clearance, 1, fwd_window=esc_win),
                           -1: self._side_free(clearance, -1, fwd_window=esc_win)}
                cand &= np.array([side_ok[1 if a > 0 else -1] for a in self.angles])
            lat = self._blocker_lateral(clearance, center)
            if lat is not None and abs(lat) >= 0.08:      # blocker clearly on one side:
                away = -1 if lat > 0 else 1               # only ever step AWAY from it
                cand &= (np.sign(self.angles) == away)
            if not cand.any():
                return None, 0.0, 0.0
            idx = np.flatnonzero(cand)
            clipped = np.clip(clearance, 0.0, self.max_depth)
            room_r = float(clipped[self.angles > 0].mean())
            room_l = float(clipped[self.angles < 0].mean())
            room = np.where(self.angles[idx] > 0, room_r, room_l) / self.max_depth
            # [REV] prefer the side with more lateral room to the (classified) corridor wall
            wroom = np.array([self._wall_room(1 if a > 0 else -1, now) for a in self.angles[idx]])
            score = fd[idx] + 100.0 * free[idx] + 5.0 * room + 5.0 * wroom
            if self.escape_dir != 0:                    # keep the side we started on
                score = score + 20.0 * (np.sign(self.angles[idx]) == self.escape_dir)
            best = int(idx[np.argmax(score)])
            self.escape_dir = 1 if self.angles[best] > 0 else -1
            self.prev_angle = float(self.angles[best])
            self.latch_dir = self.escape_dir            # keep sidestepping while passing
            self.latch_until = now + self.latch_time
            self.latch_dist = self.latch_distance
            # [REV] urgency-scaled lateral escape: closer blocker -> faster sidestep
            u = clamp((self.stop_dist - float(fd[center]))
                      / max(self.stop_dist - self.emergency_dist, 1e-3), 0.0, 1.0)
            esc_speed = min(self.max_speed,
                            self.min_speed + (self.escape_speed - self.min_speed) * u)
            self.escaping = True
            self.vy_filt = 0.0                          # [REV2] avoidance always beats centering
            self.last_avoid_time = now
            self._note_side(math.copysign(math.pi / 2, self.angles[best]), now)
            return math.copysign(math.pi / 2, self.angles[best]), esc_speed, 0.0

        self.escape_dir = 0

        # 3) Nothing fully free: creep along the most open heading, else HOLD
        if not free.any():
            if not caution.any():
                return None, 0.0, 0.0
            idx = np.flatnonzero(caution)
            best = int(idx[np.argmax(fd[idx] - 0.5 * np.abs(self.angles[idx] - self.goal))])
            self.prev_angle = float(self.angles[best])
            self.last_avoid_time = now                    # [REV] creeping = avoiding
            self.vy_filt = 0.0
            self._note_side(float(self.angles[best]), now)
            return float(self.angles[best]), self.min_speed, 0.0

        # 3b) Cost: prefer goal direction, smooth, open
        # [REV4] early positioning: a gap-trap is visible well before the centre path is
        # blocked. Drift toward the side that has room as soon as it comes into view.
        pre = 0
        if self.gap_lookahead > 0.0 and float(fd[center]) < self.gap_lookahead:
            pre = self._gap_aware_side(clearance, now)
        goal_eff = self.goal + pre * self.pre_angle
        cost = (
            self.w_goal * np.abs(self.angles - goal_eff)
            + self.w_smooth * np.abs(self.angles - self.prev_angle)
            - self.w_clear * fd / self.max_depth
        )
        # [REV2] committed avoidance side: no LEFT/RIGHT dithering. While a latch is active
        # keep its side; otherwise commit to the safer side as soon as the centre is blocked.
        stick = 0
        if pre != 0:
            stick = pre
        elif self.latch_dist > 0.0 and now < self.latch_until and self.latch_dir != 0:
            stick = self.latch_dir
        elif not free[center]:
            stick = self._pick_side(clearance, fd, now)
        if stick != 0:
            off = self.angles - self.goal
            wrong = (np.sign(off) == -stick) & (np.abs(off) > self.center_tol)
            cost = cost + self.w_side_stick * wrong
        cost[~free] = np.inf
        best = int(np.argmin(cost))
        angle = float(self.angles[best])
        fd_best = float(fd[best])
        self.prev_angle = angle

        # 4) Latch: keep sidestepping briefly after the obstacle leaves the FOV
        latch_active = (self.latch_dist > 0.0 and now < self.latch_until
                        and self.latch_dir != 0)                       # [REV]
        if abs(angle - self.goal) > self.center_tol:
            self.latch_dir = 1 if angle > self.goal else -1
            self.latch_until = now + self.latch_time
            self.latch_dist = self.latch_distance
        elif (self.latch_dist > 0.0 and now < self.latch_until
              and self.latch_dir != 0):
            probe = self._nearest(self.goal + self.latch_dir * self.latch_angle)
            if free[probe]:
                angle = float(self.angles[probe])
                fd_best = float(fd[probe])

        # 5) Wall centering only while going straight AND no avoidance is active [REV]
        vy = 0.0
        avoiding = abs(angle - self.goal) > self.center_tol or latch_active
        if avoiding:
            self.last_avoid_time = now
            self.vy_filt = 0.0                          # [REV2] obstacle avoidance wins at once
            self._cgate = 'avoiding'
        else:
            # [REV2] centre only when the chosen path is fully clear; smooth the command
            vy_t = self._centering(clearance, free, now) if fd_best >= self.eff_block else 0.0
            if fd_best < self.eff_block:
                self._cgate = 'fd<block'
            self.vy_filt += self.center_alpha * (vy_t - self.vy_filt)
            vy = self.vy_filt
        self._note_side(angle, now)

        # 6) Slow down as free distance along the chosen path shrinks
        span = max(1.5 * self.eff_block - self.stop_dist, 1e-3)
        frac = clamp((fd_best - self.stop_dist) / span, 0.0, 1.0)
        speed = self.min_speed + (self.max_speed - self.min_speed) * frac
        return angle, speed, vy

    def _pick_side(self, clearance, fd, now):
        """
        [REV2] Safer side (+1 right / -1 left) to avoid a blocked centre path: away from the
        blocker if it is clearly one-sided, else the side with more room (open headings,
        wall room), penalising sides with remembered obstacles or unknown space.
        """
        center = len(clearance) // 2
        gap_side = self._gap_aware_side(clearance, now)       # [REV4]
        if gap_side != 0:
            return gap_side
        lat = self._blocker_lateral(clearance, center)
        if lat is not None and abs(lat) >= 0.08:
            return -1 if lat > 0 else 1
        best, best_score = 0, -1e9
        for sgn in (-1, 1):
            sel = (self.angles * sgn) >= self.escape_min_angle
            if not sel.any():
                continue
            score = 5.0 * float(np.mean(np.clip(fd[sel], 0.0, self.max_depth))) / self.max_depth
            score += 3.0 * self._wall_room(sgn, now)
            if not self._side_free(clearance, sgn):
                score -= 10.0
            if np.any(clearance[sel] < 0.0):
                score -= 3.0
            if sgn == self.last_side:
                score += 0.5
            if score > best_score:
                best, best_score = sgn, score
        return best

    def _side_guard(self, clearance):
        """
        [REV4] Reactive lateral push away from anything beside the body. Planning keeps
        safety_radius, but wind, sensor latency or velocity lag can still close a thin
        margin; this term opposes the drift (left and right pushes cancel in a tight gap).
        Returns a body-frame lateral velocity (+right), capped at guard_max.
        """
        if self.guard_gain <= 0.0:
            return 0.0
        f, y, _, _, _ = self._obstacle_points(clearance)
        beside = (f > -(self.body_half + 0.1)) & (f < self.guard_ahead)
        if not beside.any():
            return 0.0
        left = -y[beside & (y < 0.0)]
        right = y[beside & (y > 0.0)]
        d_left = float(left.min()) if left.size else 9.0
        d_right = float(right.min()) if right.size else 9.0
        push = max(0.0, self.guard_dist - d_left) - max(0.0, self.guard_dist - d_right)
        return clamp(self.guard_gain * push, -self.guard_max, self.guard_max)

    def _passing_rooms(self, clearance, now):
        """
        [REV4] Free width beside the obstacle that blocks the centre path.
        Returns None, or (room_left, room_right, along_nearest, lat_left_edge, lat_right_edge);
        a room is None when the wall on that side is unknown (then it is not a constraint).
        Edges are lateral offsets (+right) of the blocker cluster; rooms are measured from the
        blocker edge to the (classified) corridor wall.
        """
        f, y, unc, amin, _ = self._obstacle_points(clearance)
        a = float(self.angles[len(clearance) // 2])
        along = f * math.cos(a) + y * math.sin(a)
        lat = -f * math.sin(a) + y * math.cos(a)
        hit = (along > amin) & (np.abs(lat) - unc < self.safety_radius)
        if not hit.any():
            return None
        a0 = float(along[hit].min())
        cl = hit & (along < a0 + self.cluster_depth)
        lo, hi = float(lat[cl].min()), float(lat[cl].max())
        wl = self.wall_lat[-1] if self._wall_valid(-1, now) else None
        wr = self.wall_lat[1] if self._wall_valid(1, now) else None
        width = self.width_est if self.width_est else self.corridor_width
        if width and width > 0.0:
            if wl is None and wr is not None:
                wl = width - wr
            if wr is None and wl is not None:
                wr = width - wl
        room_l = None if wl is None else wl + lo          # lo <= 0 means left of the drone
        room_r = None if wr is None else wr - hi
        return room_l, room_r, a0, lo, hi

    def _gap_aware_side(self, clearance, now):
        """
        [REV4] Pick the side that actually has room to pass, not just the side the drone
        happens to be on. Returns +1/-1, or 0 = no clear preference (use the old rule).
        Crossing in front of the blocker is allowed only if the bearing needed is reachable.
        """
        rooms = self._passing_rooms(clearance, now)
        if rooms is None:
            return 0
        room_l, room_r, a0, lo, hi = rooms
        ok = {}
        for side, room in ((-1, room_l), (1, room_r)):
            need = self.pass_gap - (self.pass_gap_hyst if side == self.last_side else 0.0)
            ok[side] = room is None or room >= need
        if ok[-1] == ok[1]:
            return 0                                        # both fine or both too tight
        side = -1 if ok[-1] else 1
        tgt = (lo - self.safety_radius) if side < 0 else (hi + self.safety_radius)
        shift = max(0.0, -tgt) if side < 0 else max(0.0, tgt)   # lateral displacement needed
        limit = self.cross_max
        if self.gap_side == side and now < self.gap_until:
            limit += self.cross_commit                      # already committed: do not flip back
        if shift > 0.0 and math.atan2(shift, max(a0, 0.05)) > limit:
            return 0                                        # cannot cross in time: old rule
        self.gap_side, self.gap_until = side, now + self.gap_commit_time
        return side

    def _note_side(self, angle, now):
        """[REV] Record left<->right direction flips for oscillation (deadlock) detection."""
        if abs(angle - self.goal) <= self.center_tol:
            return
        side = 1 if angle > self.goal else -1
        if self.last_side != 0 and side != self.last_side:
            self.side_flips.append((now, self.pos_x, self.pos_y))
        self.last_side = side

    # ---- [REV] wall vs obstacle ------------------------------------------------
    def _classify_walls(self, clearance):
        """
        Conservative wall/obstacle heuristic on the EXISTING sector representation.

        WALL     = a run of >= wall_min_sectors adjacent sectors on one side whose lateral
                   offset (r*sin) stays within wall_lat_tol AND whose forward extent
                   (r*cos) spans >= wall_min_span, i.e. a long line parallel to the heading.
        OBSTACLE = every other sector with a return inside wall_max_range (box faces, poles,
                   short side pillars: lateral offset varies or there is no forward extent).
        UNKNOWN  = no-data sector. Never used as wall.
        The planner always sees ALL points; the classification only decides what the
        centering controller may centre on. Limits: assumes yaw within ~8 deg of the corridor
        axis; a long flat object parallel to the heading is indistinguishable from a wall.
        Returns {side: lateral distance | None}, side -1 = left, +1 = right.
        """
        n = len(clearance)
        cls = np.zeros(n, dtype=np.int8)          # 0 free/far, 1 wall, 2 obstacle, 3 unknown
        cls[clearance < 0.0] = 3
        cls[(clearance >= 0.0) & (clearance < self.wall_max_range)] = 2
        known = clearance > self.min_depth
        near = known & (clearance < self.wall_max_range)
        r = np.where(known, clearance, np.nan)
        f = r * np.cos(self.angles)
        y = r * np.sin(self.angles)
        raw = {-1: None, 1: None}
        yaw_ok = self._yaw_consistent()      # [REV2] uncertain geometry -> stay "obstacle"
        for side in (-1, 1):
            if not yaw_ok:
                continue
            order = [int(i) for i in np.argsort(np.abs(self.angles))
                     if side * self.angles[i] >= self.wall_min_angle]
            ok = [bool(near[i]) and abs(y[i]) <= self.center_valid_lat for i in order]
            best, m, j = None, len(order), 0
            while j < m:
                if not ok[j]:
                    j += 1
                    continue
                lo = hi = abs(y[order[j]])
                fmin = fmax = f[order[j]]
                k = j
                while k + 1 < m and ok[k + 1]:
                    v = abs(y[order[k + 1]])
                    if max(hi, v) - min(lo, v) > self.wall_lat_tol:
                        break
                    lo, hi = min(lo, v), max(hi, v)
                    fmin, fmax = min(fmin, f[order[k + 1]]), max(fmax, f[order[k + 1]])
                    k += 1
                run = order[j:k + 1]
                if len(run) >= self.wall_min_sectors and (fmax - fmin) >= self.wall_min_span:
                    lat = float(np.median([abs(y[i]) for i in run]))
                    key = (len(run), -lat)
                    if best is None or key > best[0]:
                        best = (key, run, lat)
                j = k + 1
            if best is not None:
                cls[best[1]] = 1
                raw[side] = best[2]
        self.sector_class = cls
        return raw

    def _update_walls(self, clearance, now):
        """Temporal confirmation + low-pass of the per-frame wall estimates."""
        raw = self._classify_walls(clearance)
        for s in (-1, 1):
            lat = raw[s]
            est0 = self.wall_lat[s]
            if lat is not None and est0 is not None and abs(lat - est0) > self.wall_jump:
                lat = None                   # [REV2] sudden jump = local obstacle / box face, not wall
            if lat is not None:
                self.wall_conf[s] = min(self.wall_conf[s] + 1, 10)
                est = self.wall_lat[s]
                self.wall_lat[s] = lat if est is None else est + self.wall_alpha * (lat - est)
                self.wall_seen[s] = now
            else:
                self.wall_conf[s] = max(self.wall_conf[s] - 2, 0)
                if now - self.wall_seen[s] > self.wall_lost_time:
                    self.wall_lat[s] = None
        if self._wall_valid(-1, now) and self._wall_valid(1, now):
            w = self.wall_lat[-1] + self.wall_lat[1]
            if 2.0 < w < 6.0:
                self.width_est = w if self.width_est is None else self.width_est + 0.1 * (w - self.width_est)

    def _wall_valid(self, side, now):
        return (self.wall_lat[side] is not None and self.wall_conf[side] >= self.wall_confirm
                and now - self.wall_seen[side] <= self.wall_lost_time)

    def _wall_room(self, side, now):
        """0..1 lateral room to the wall on `side`; neutral 0.5 when unknown."""
        if not self._wall_valid(side, now):
            return 0.5
        return clamp(self.wall_lat[side] / max(self.center_valid_lat, 0.1), 0.0, 1.0)

    def _centering(self, clearance, free, now):
        """
        Lateral speed (body +Y = right) that equalises left/right WALL distance.
        [REV] uses only wall-classified, temporally confirmed, low-passed estimates;
        dead-band; ramps in after avoidance; never towards a blocked / remembered /
        unknown side. Obstacle avoidance always has priority (caller skips this while
        avoiding).
        """
        vl, vr = self._wall_valid(-1, now), self._wall_valid(1, now)
        self._cerr = 0.0
        self._cgate = 'ok'
        scale = 1.0
        if vl and vr:
            if abs(self.wall_lat[1] + self.wall_lat[-1] - self.corridor_width) > self.corridor_width_tol:
                self._cgate = 'width'
                return 0.0                    # [REV3] implausible width: not a real corridor pair
            err = self.wall_lat[1] - self.wall_lat[-1]
            self._cerr = err
            lat_min = min(self.wall_lat[1], self.wall_lat[-1])
        elif self.single_wall and (vl or vr):
            width = self.width_est if self.width_est else self.corridor_width
            if width <= 0.0:
                return 0.0
            if vl:
                err, lat_min = width - 2.0 * self.wall_lat[-1], self.wall_lat[-1]
            else:
                err, lat_min = 2.0 * self.wall_lat[1] - width, self.wall_lat[1]
            tgt = 1 if err > 0 else -1
            if np.any(clearance[(self.angles * tgt) >= self.center_min_angle] < 0.0):
                return 0.0                    # target side contains unknown space
            scale = 0.5
        else:
            self._cgate = 'nowall'
            return 0.0
        if abs(err) <= self.center_deadband:
            self._cgate = 'deadband'
            return 0.0
        err = math.copysign(abs(err) - self.center_deadband, err)
        ramp = clamp((now - self.last_avoid_time - self.center_resume_delay)
                     / max(self.center_ramp_time, 1e-3), 0.0, 1.0)
        vmax = self.center_max_close if lat_min < self.wall_warn_lat else self.center_max
        vy = clamp(self.center_kp * err * scale * ramp, -vmax, vmax)
        self._cramp = ramp
        if ramp <= 0.0:
            self._cgate = 'ramp'
        if vy != 0.0:
            s = 1 if vy > 0 else -1
            probe = self._nearest(math.copysign(math.radians(20.0), vy))
            fd_probe = float(self._fd_last[probe]) if self._fd_last is not None else 0.0
            if fd_probe < self.center_probe_dist:
                self._cgate = 'probe'
                return 0.0
            if not self._side_free(
                    clearance, s, self.centering_extra_lat, self.center_extra_fwd):
                self._cgate = 'sidefree'
                return 0.0                   # never centre toward blocked / remembered obstacle
            if np.any(clearance[(self.angles * s) >= self.center_min_angle] < 0.0):
                self._cgate = 'unknown'
                return 0.0                   # [REV2] never toward unknown space
            near_obst = ((self.sector_class == 2) & (self.angles * s > 0.0)
                         & (clearance >= 0.0) & (clearance < self.eff_block))
            if near_obst.any():
                self._cgate = 'nearobst'
                return 0.0                   # [REV2] never toward a non-wall obstacle
        return vy

    def _update_stuck(self, angle, speed, now):
        moving = angle is not None and speed > 0.0
        if moving:
            self.hold_since = None
            if self.eff_block < self.block_dist:
                self.eff_block = min(self.block_dist, self.eff_block + self.restore_step)
            return
        if self.hold_since is None:
            self.hold_since = now
        elif ((now - self.hold_since) > self.stuck_timeout
              and (now - self.last_relax) >= self.relax_interval
              and self.eff_block > self.min_block_dist):
            self.eff_block = max(self.min_block_dist, self.eff_block - self.relax_step)
            self.blocked[:] = False
            self.clear_count[:] = 0
            self.last_relax = now
            self.get_logger().warn(
                f'Stuck: relaxing obstacle threshold to {self.eff_block:.2f} m '
                f'(floor {self.min_block_dist:.2f} m)')

    # ---- [REV] deadlock / trapped-state recovery ---------------------------------
    def _track_motion(self, now, healthy):
        """Breadcrumb trail of the path actually flown + progress history."""
        p = (self.pos_x, self.pos_y)
        if (self.trail_last is None
                or math.hypot(p[0] - self.trail_last[0], p[1] - self.trail_last[1])
                >= self.trail_spacing):
            self.trail.append(p)
            self.trail_last = p
            if len(self.trail) > self.trail_max:
                del self.trail[0]
        if healthy:
            self.pos_hist.append((now, p[0], p[1]))
            while self.pos_hist and now - self.pos_hist[0][0] > self.fwd_window:
                self.pos_hist.pop(0)
        else:
            self.pos_hist.clear()           # perception problems are not "no progress"

    def _deadlocked(self, now):
        """'' or a reason: almost no movement for progress_window, or repeated L/R flips."""
        h = self.pos_hist
        recent = [e for e in h if now - e[0] <= self.progress_window]
        if len(recent) > 2 and now - recent[0][0] >= 0.9 * self.progress_window:
            far = max(math.hypot(x - self.pos_x, y - self.pos_y) for _, x, y in recent)
            if far < self.progress_min:
                return 'no progress'
        if len(h) > 2 and now - h[0][0] >= 0.9 * self.fwd_window:      # [REV2]
            c, sn = math.cos(self.hold_yaw), math.sin(self.hold_yaw)
            fwd = (self.pos_x - h[0][1]) * c + (self.pos_y - h[0][2]) * sn
            if fwd < self.fwd_min:
                return 'insufficient forward progress'
        self.side_flips = [f for f in self.side_flips if now - f[0] <= self.osc_window]
        if len(self.side_flips) >= self.osc_flips:
            f0 = self.side_flips[0]
            if math.hypot(f0[1] - self.pos_x, f0[2] - self.pos_y) < 0.6:
                return 'direction oscillation'
        return ''

    def _rear_clear(self, dist):
        """
        There is NO rear sensor. Backing off is allowed only through space the drone has
        already flown through (continuous breadcrumb trail straight behind it, within
        trail_lateral_tol) and with no remembered obstacle point behind it.
        """
        if len(self.trail) < 2:
            return False
        c, sn = math.cos(self.hold_yaw), math.sin(self.hold_yaw)
        t = np.array(self.trail)
        dx, dy = t[:, 0] - self.pos_x, t[:, 1] - self.pos_y
        f, y = dx * c + dy * sn, -dx * sn + dy * c
        on_path = np.abs(y) < self.trail_lat_tol
        s = self.trail_spacing
        for k in range(1, int(math.ceil(dist / s)) + 1):
            if not np.any(on_path & (np.abs(f + k * s) < s)):
                return False
        mf, my = self._memory_body()
        if mf.size:
            behind = ((mf < 0.0) & (mf > -(dist + 0.5))
                      & (np.abs(my) < self.safety_radius + self.body_half))
            if behind.any():
                return False
        return True

    def _recovery_update(self, now, healthy, hold_for, angle):
        """
        STOP -> (BACKOFF along the flown path if safe) -> EVAL -> normal navigation.
        Max `recovery_max_attempts` per stuck location; afterwards the drone simply keeps
        hovering (HOLD / STUCK) and the pilot override applies. Returns '' | R_*.
        """
        if not self.recovery_enabled or self.state != S_NAVIGATE:
            self.rec_state = ''
            return ''
        rs = self.rec_state
        if rs == '':
            if not healthy:
                return ''
            if (self.rec_pos is not None
                    and math.hypot(self.pos_x - self.rec_pos[0], self.pos_y - self.rec_pos[1]) > 1.5):
                self.rec_attempts = 0                 # real progress since last recovery
            if self.rec_attempts >= self.recovery_max_attempts:
                return ''
            urgent, why = False, ''
            if angle is None and self.center_fd < self.emergency_dist:
                urgent, why = True, 'blocker inside emergency distance, no escape side'
            elif now >= self.rec_cooldown_until:
                if hold_for > self.retreat_after:
                    why = 'HOLD too long'
                else:
                    why = self._deadlocked(now)
            if not why:
                return ''
            self.rec_state, self.rec_t0, self.rec_urgent = R_STOP, now, urgent
            self.rec_escaped = False
            self.rec_attempts += 1
            self.rec_pos = (self.pos_x, self.pos_y)
            self.get_logger().warn(
                f'Recovery {self.rec_attempts}/{self.recovery_max_attempts}: {why} -> STOP')
            return R_STOP
        if rs == R_STOP:
            if now - self.rec_t0 >= (0.3 if self.rec_urgent else self.recovery_stop_time):
                back = self.retreat_speed * self.retreat_time
                if (self.retreat_enabled and healthy and self._rear_clear(back + 0.2)):
                    self.rec_state, self.rec_t0 = R_BACK, now
                    self.get_logger().warn('Recovery: backing off along flown path')
                    return R_BACK
                self.get_logger().warn('Recovery: back-off not safe/enabled -> re-evaluate')
                self._recovery_enter_eval(now)
                return R_EVAL
            return R_STOP
        if rs == R_BACK:
            if (not healthy) or now - self.rec_t0 >= self.retreat_time:
                self._recovery_enter_eval(now)
                return R_EVAL
            return R_BACK
        if rs == R_EVAL:
            if now - self.rec_t0 >= self.recovery_eval_time:
                if angle is None and not self.rec_escaped and healthy:
                    side = self._safest_escape_side()
                    if side != 0:                       # [REV2] guarded lateral escape step
                        self.rec_state, self.rec_t0, self.rec_side = R_ESC, now, side
                        self.rec_escaped = True
                        self.get_logger().warn(
                            f'Recovery: safest escape = {"RIGHT" if side > 0 else "LEFT"}')
                        return R_ESC
                    self.get_logger().warn('Recovery: no safe escape -> HOLD')
                self.rec_state = ''
                self.rec_cooldown_until = now + self.retreat_cooldown
                return ''
            return R_EVAL
        if rs == R_ESC:
            if ((not healthy) or not self._escape_side_ok(self.rec_side)
                    or now - self.rec_t0 >= self.rec_esc_time or angle is not None):
                self._recovery_enter_eval(now)          # RE-EVALUATE before resuming
                return R_EVAL
            return R_ESC
        self.rec_state = ''
        return ''

    def _escape_side_ok(self, side):
        """[REV2] Lateral escape toward `side` is allowed only through KNOWN, clear space."""
        cl = self.clearance
        sel = (self.angles * side) >= self.escape_min_angle
        if not sel.any() or np.any(cl[sel] < self.creep_dist):   # unknown (-1) also fails
            return False
        return self._side_free(cl, side, self.center_extra_lat,
                               fwd_window=self.body_half + 0.1)

    def _safest_escape_side(self):
        """[REV2] Safest escape side (+1/-1) or 0 = none (then HOLD, never a random move)."""
        center = len(self.clearance) // 2
        lat = self._blocker_lateral(self.clearance, center)
        best, best_score = 0, -1e9
        for side in (-1, 1):
            if lat is not None and abs(lat) >= 0.08 and side == (1 if lat > 0 else -1):
                continue                                 # never toward the blocker
            if not self._escape_side_ok(side):
                continue
            sel = (self.angles * side) >= self.escape_min_angle
            score = float(np.mean(np.clip(self.clearance[sel], 0.0, self.max_depth)))
            score += 3.0 * self._wall_room(side, self.now_s())
            if side == self.last_side:
                score += 0.5
            if score > best_score:
                best, best_score = side, score
        return best

    def _recovery_enter_eval(self, now):
        """Fresh start for the planner: forget latched decisions, re-evaluate depth."""
        self.rec_state, self.rec_t0 = R_EVAL, now
        self.hold_since = None
        self.blocked[:] = False
        self.clear_count[:] = 0
        self.escape_dir = 0
        self.latch_dir, self.latch_dist = 0, 0.0
        self.prev_angle = self.goal
        self.side_flips = []
        self.pos_hist.clear()

    def label(self, angle, speed):
        if angle is None or speed <= 0.0:
            return 'HOLD'
        if abs(angle - self.goal) <= self.center_tol:
            return 'FORWARD'
        return 'LEFT' if angle < self.goal else 'RIGHT'

    # ------------------------------------------------------------------
    # CONTROL
    # ------------------------------------------------------------------
    def altitude_vz(self):
        err = (-self.alt_target) - self.current_z      # [VERT] == takeoff_alt unless the vertical layer moved it
        return clamp(self.kp_alt * err, -self.max_climb, self.max_descend)

    def body_to_ned(self, vx_b, vy_b):
        c, s = math.cos(self.hold_yaw), math.sin(self.hold_yaw)
        return vx_b * c - vy_b * s, vx_b * s + vy_b * c

    def slew(self, target_n, target_e):
        """Gentle acceleration, hard braking."""
        dn, de = target_n - self.cur_vn, target_e - self.cur_ve
        dist = math.hypot(dn, de)
        braking = (dn * self.cur_vn + de * self.cur_ve) < 0.0     # slowing or reversing
        limit = (self.max_decel if braking else self.max_accel) * self.dt
        if dist > limit:
            dn, de = dn * limit / dist, de * limit / dist
        self.cur_vn += dn
        self.cur_ve += de
        return self.cur_vn, self.cur_ve

    def control_loop(self):
        try:
            self._control_step()
        except Exception as e:  # noqa: BLE001
            # An uncaught exception would kill spin() and the offboard heartbeat.
            self.get_logger().error(f'Control loop error: {e}', throttle_duration_sec=1.0)
            self._safe_hold_publish()

    def _safe_hold_publish(self):
        if not self.flight_enabled or self.state in (S_WAIT, S_ABORTED):
            return
        try:
            vn, ve = self.slew(0.0, 0.0)
            self.publish_offboard_mode()
            self.publish_velocity(vn, ve, 0.0, self.hold_yaw)
        except Exception:  # noqa: BLE001
            pass

    def _control_step(self):
        self.cycle += 1
        now = self.now_s()
        depth_ok = (now - self.last_depth_time) < self.depth_timeout
        pos_ok = ((now - self.last_position_time) < self.position_timeout
                  and self.pose_valid)

        if pos_ok:
            self._lock_yaw()                   # [REV2] establish hold_yaw before memory is used

        if self.depth_received and not depth_ok:
            self.get_logger().warn('Depth stream stale -> HOLD', throttle_duration_sec=2.0)
            self.good_frames = 0               # [REV] need fresh good frames to resume

        # [REV] move only on fresh depth that has been good for depth_resume_frames frames
        if depth_ok and self.good_frames >= self.depth_resume_frames:
            angle, speed, vy_center = self.nav_angle, self.nav_speed, self.nav_vy
        else:
            angle, speed, vy_center = None, 0.0, 0.0
        decision = self.label(angle, speed)

        # Consume the avoidance latch by forward distance actually travelled.
        if self.latch_dist > 0.0:
            fwd = (self.cur_vn * math.cos(self.hold_yaw)
                   + self.cur_ve * math.sin(self.hold_yaw))
            self.latch_dist = max(0.0, self.latch_dist - max(fwd, 0.0) * self.dt)

        # STUCK reporting + [REV] conservative recovery (STOP -> back off -> re-evaluate)
        hold_for = (now - self.hold_since) if (depth_ok and self.hold_since is not None) else 0.0
        healthy = (depth_ok and not self.percep_hold
                   and self.good_frames >= self.depth_resume_frames)
        if self.state == S_NAVIGATE and pos_ok:
            self._track_motion(now, healthy)
        rec = self._recovery_update(now, healthy, hold_for, angle)
        retreating = rec == R_BACK
        if rec == R_BACK:
            decision = 'RETREAT'
        elif rec == R_STOP:
            decision = 'RECOVER_STOP'
        elif rec == R_EVAL:
            decision = 'RECOVER_EVAL'
        elif rec == R_ESC:
            decision = 'RECOVER_ESCAPE'
        elif hold_for > self.stuck_timeout:
            decision = 'STUCK'
            self.get_logger().warn('STUCK: no safe heading', throttle_duration_sec=3.0)

        msg = String()
        msg.data = decision
        self.decision_pub.publish(msg)
        if self.cycle % max(1, int(self.control_rate // 2)) == 0:     # [REV] 2 Hz diagnostics
            sym = {0: '.', 1: 'W', 2: 'O', 3: '?'}
            d = String()
            wl, wr = self.wall_lat[-1], self.wall_lat[1]
            d.data = (f'conf={self.percep_factor:.2f} hold={int(self.percep_hold)} '
                      f'reason="{self.percep_reason}" '
                      f'wallL={"-" if wl is None else f"{wl:.2f}"} '
                      f'wallR={"-" if wr is None else f"{wr:.2f}"} '
                      f'rec={rec or "-"} attempts={self.rec_attempts} '
                      f'cgate={self._cgate} cerr={self._cerr:+.2f} vy={self.vy_filt:+.3f} '
                      f'sectors=' + ''.join(sym[int(c)] for c in self.sector_class))
            self.diag_pub.publish(d)
        if decision != self.last_decision:
            self.get_logger().info(
                'Clearance L->R: ' + ' '.join(f'{c:.1f}' for c in self.clearance))
            self.get_logger().info(f'>>> DEPTH DECISION: {decision} <<<')
            self.last_decision = decision

        if not self.flight_enabled:
            self._periodic_log(decision, 0.0, 0.0, 0.0)
            return

        if self.state == S_ABORTED:
            return

        if self.state == S_WAIT:
            # [REV] do not arm / take off before depth is valid
            if (self.depth_received and self.position_received
                    and self.good_frames >= self.depth_resume_frames):
                self._lock_yaw(force=True)
                if not self.yaw_locked:
                    self.get_logger().warn('Waiting for a valid heading.',
                                           throttle_duration_sec=2.0)
                    return
                self.get_logger().info(
                    f'Sensors ready. Locked heading = {math.degrees(self.hold_yaw):.1f} deg')
                self.state = S_PREFLIGHT
                self.preflight_count = 0
            return

        # Pilot took over or vehicle disarmed -> stop commanding, do not fight.
        if (self.state in (S_TAKEOFF, S_NAVIGATE) and self.status_received
                and (not self.offboard or not self.armed)):
            self.get_logger().error(
                'Left OFFBOARD or disarmed. Controller aborted (not re-requesting).')
            self.state = S_ABORTED
            return

        target_n = target_e = vz = 0.0

        if self.state == S_PREFLIGHT:
            self.preflight_count += 1
            if self.preflight_count >= self.preflight_cycles:
                self.state = S_ARMING

        elif self.state == S_ARMING:
            if self.armed and self.offboard:
                self.get_logger().info('Armed and in OFFBOARD. Taking off.')
                self.state = S_TAKEOFF
            else:
                if self.cycle % int(self.control_rate) == 0:
                    self.request_offboard()
                    self.arm()
                if not self.status_received:
                    self.get_logger().warn(
                        f'No data on {self.status_topic}: cannot confirm arm/offboard '
                        '(check topic name with `ros2 topic list`).',
                        throttle_duration_sec=5.0)

        elif self.state == S_TAKEOFF:
            vz = self.altitude_vz() if pos_ok else 0.0
            if pos_ok and abs(self.current_z + self.takeoff_alt) < self.alt_tol:
                self.get_logger().info('Takeoff altitude reached. Navigating.')
                self.state = S_NAVIGATE

        elif self.state == S_NAVIGATE:
            vz = self.altitude_vz() if pos_ok else 0.0
            vx_b = vy_b = 0.0
            if rec in (R_STOP, R_EVAL):
                pass                            # [REV] recovery: stand still, re-evaluate
            elif rec == R_ESC and pos_ok and depth_ok:
                vy_b = self.rec_side * self.rec_esc_speed      # [REV2] guarded lateral escape
                self.last_avoid_time = now
                self.vy_filt = 0.0
                self.latch_dir = self.rec_side
                self.latch_until = now + self.latch_time
                self.latch_dist = self.latch_distance
            elif retreating and pos_ok:
                vx_b = -self.retreat_speed
            elif depth_ok and pos_ok and self.yaw_locked and angle is not None and speed > 0.0:
                vx_b = speed * math.cos(angle)
                vy_b = speed * math.sin(angle) + vy_center
                norm = math.hypot(vx_b, vy_b)
                if norm > self.max_speed:
                    vx_b, vy_b = vx_b * self.max_speed / norm, vy_b * self.max_speed / norm
            self.cmd_body = (vx_b, vy_b)
            target_n, target_e = self.body_to_ned(vx_b, vy_b)

        vn, ve = self.slew(target_n, target_e)
        self._periodic_log(decision, vn, ve, vz)

        self.publish_offboard_mode()
        self.publish_velocity(vn, ve, vz, self.hold_yaw)

    def _periodic_log(self, decision, vn, ve, vz):
        if self.cycle % int(self.control_rate) == 0:
            self.get_logger().info(
                f'[{self.state}] {decision} | thr={self.eff_block:.2f} | '
                f'conf={self.percep_factor:.2f} | '
                f'CMD(NED): Vn={vn:.2f} Ve={ve:.2f} Vz={vz:.2f} | z={self.current_z:.2f}')

    # ------------------------------------------------------------------
    # PX4 INTERFACE
    # ------------------------------------------------------------------
    def publish_offboard_mode(self):
        msg = OffboardControlMode()
        msg.position = False
        msg.velocity = True
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = self.timestamp()
        self.offboard_mode_pub.publish(msg)

    def publish_velocity(self, vn, ve, vz, yaw):
        # Local NED (+X north, +Y east, +Z down), NOT body frame.
        msg = TrajectorySetpoint()
        nan = float('nan')
        msg.velocity = [float(vn), float(ve), float(vz)]
        msg.position = [nan, nan, nan]
        msg.acceleration = [nan, nan, nan]
        msg.jerk = [nan, nan, nan]
        msg.yaw = float(yaw)
        msg.yawspeed = nan
        msg.timestamp = self.timestamp()
        self.trajectory_pub.publish(msg)

    def vehicle_command(self, command, param1=0.0, param2=0.0):
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = float(param1)
        msg.param2 = float(param2)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = self.timestamp()
        self.vehicle_command_pub.publish(msg)

    def request_offboard(self):
        self.vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
        self.get_logger().info('OFFBOARD mode command sent')

    def arm(self):
        self.vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
        self.get_logger().info('ARM command sent')

    def land(self):
        self.vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        self.get_logger().info('LAND command sent')


def main(args=None):
    rclpy.init(args=args)
    node = CorridorFlight()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            if (node.flight_enabled and node.land_on_exit
                    and node.state in (S_TAKEOFF, S_NAVIGATE)):
                node.land()
        except Exception:  # noqa: BLE001
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
