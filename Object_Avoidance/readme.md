# Object_Avoidance

Depth-camera obstacle avoidance for corridor flight: lateral avoidance, wall centring, and a vertical over/under layer.
One file, one ROS 2 node: `avoidance_standalone.py`. No Gazebo and no colcon package are needed.

## Requirements
- Ubuntu 22.04, ROS 2 Humble (sourced), `px4_msgs` built in the same workspace
- Orbbec Gemini 330 ROS 2 driver publishing depth (32FC1 in metres, or 16UC1 in mm)
- Flight controller speaking the PX4 offboard interface (`/fmu/in/...`, `/fmu/out/...`). A flight controller that does not
  speak PX4 needs an adapter for the 4 topics (position, status, setpoint, vehicle command); the avoidance logic is unchanged.

## Run
Bench / dry run (the default; props off; nothing is sent to the flight controller):

    python3 avoidance_standalone.py

Flight (arms, takes off to `takeoff_altitude`, flies the corridor):

    python3 avoidance_standalone.py --ros-args -p flight_enabled:=true

Any parameter can be overridden, for example `-p max_speed:=0.4 -p depth_topic:=/camera/depth/image_raw`.
Decisions are on `/depth_navigation/decision`; diagnostics (wallL/wallR, why wall centring is idle) on `/depth_navigation/diagnostics`.

## Key parameters (defaults)
| Parameter | Default | Meaning |
|---|---|---|
| `flight_enabled` | false | false = dry run |
| `depth_topic` | /camera/depth/image_raw | depth image |
| `hfov_deg` | 86 | depth horizontal field of view (Gemini 330) |
| `takeoff_altitude` | 2.0 | cruise height above the floor, m |
| `vert_ground_offset` | 0.0 | AGL = -local_z + this; measure it on the real drone |
| `max_speed` | 0.4 | m/s; everything was validated at this speed only |
| `corridor_width` / `corridor_width_tol` | 1.65 / 0.4 | corridor width used by wall centring, m |
| `safety_radius` / `pass_gap` | 0.35 / 0.85 | clearance around the drone / smallest gap it will pass, m |
| `vertical_enabled` / `vert_release_margin` | true / 1.0 | over/under layer; distance flown past an obstacle before returning to cruise, m |

## Validation (Gazebo Fortress, 1.65 m corridor, 0.4 m/s)
In the final sweep 19 of 20 world cases passed and none collided (closest approach at least +0.178 m beyond a 0.336 m body radius);
the remaining case stopped safely. Covered: slalom pillars, flush blocks, centre blocks, beams, windows, low walls,
a 12 m under-pass, and safe stalls where there is no room.

## First flight checklist
1. Dry run on the bench, props off; check `/depth_navigation/decision` reacts to a hand in front of the camera.
2. Confirm `hfov_deg` against the camera's `camera_info`.
3. Tethered hover at 2.0 m: check the altitude estimate is stable and measure the real height; set `vert_ground_offset` if needed.
4. Fly `flight_enabled:=true` at 0.4 m/s in an empty corridor, then add one obstacle at a time.
