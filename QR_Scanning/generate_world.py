import random
import os
import json
import math

# Output locations
WORLD_FILE = os.path.expanduser("~/PX4-Autopilot/Tools/simulation/gz/worlds/random_qr_world.sdf")
DATA_FILE = os.path.expanduser("~/world_data.json")
QR_DIR = os.path.expanduser("~/qr_codes")  # Folder containing 1.jpeg - 5.jpeg

AREA_WIDTH = 30.0
AREA_HEIGHT = 40.0
RED_ZONE_AREA = AREA_WIDTH * AREA_HEIGHT * 0.15  # 180 sq meters

DRONE_START_X = -18.0
DRONE_START_Y = -13.0
SAFE_RADIUS = 3.0           # Safe distance from drone spawn
MIN_DIST_BETWEEN_QRS = 2.5  # Min distance between any two QR targets to prevent overlap

# 1. Generate Red Zone Dimensions
red_w = random.uniform(10.0, 18.0)
red_h = RED_ZONE_AREA / red_w

# 2. Position Red Zone safely away from drone start position
def is_safe_from_drone(rx, ry, rw, rh):
    in_x = (rx - rw/2 - SAFE_RADIUS) < DRONE_START_X < (rx + rw/2 + SAFE_RADIUS)
    in_y = (ry - rh/2 - SAFE_RADIUS) < DRONE_START_Y < (ry + rh/2 + SAFE_RADIUS)
    return not (in_x and in_y)

while True:
    rx = random.uniform(-AREA_WIDTH/2 + red_w/2, AREA_WIDTH/2 - red_w/2)
    ry = random.uniform(-AREA_HEIGHT/2 + red_h/2, AREA_HEIGHT/2 - red_h/2)
    if is_safe_from_drone(rx, ry, red_w, red_h):
        break

# 3. Generate 5 Unique, Non-Overlapping Labeled QR Codes
available_ids = list(range(1, 6))  # Exactly [1, 2, 3, 4, 5]
random.shuffle(available_ids)      # Shuffle to assign each image uniquely without duplicates

qrs = []

def is_in_red_zone(qx, qy):
    return (rx - red_w/2 <= qx <= rx + red_w/2) and (ry - red_h/2 <= qy <= ry + red_h/2)

def is_too_close_to_other_qrs(qx, qy, existing_qrs):
    for q in existing_qrs:
        dist = math.hypot(qx - q['x'], qy - q['y'])
        if dist < MIN_DIST_BETWEEN_QRS:
            return True
    return False

for qr_id in available_ids:
    img_name = f"{qr_id}.jpeg"
    img_path = os.path.join(QR_DIR, img_name)
    
    while True:
        qx = random.uniform(-AREA_WIDTH/2 + 0.5, AREA_WIDTH/2 - 0.5)
        qy = random.uniform(-AREA_HEIGHT/2 + 0.5, AREA_HEIGHT/2 - 0.5)
        
        # Ensure target is outside Red Zone and at least 2.5m away from other QR codes
        if not is_in_red_zone(qx, qy) and not is_too_close_to_other_qrs(qx, qy, qrs):
            qrs.append({
                "id": qr_id,
                "label": img_name,
                "image_path": img_path,
                "x": qx,
                "y": qy
            })
            break

# Sort by ID so JSON output remains neatly ordered 1..5
qrs.sort(key=lambda item: item["id"])

# 4. Generate SDF World Content
sdf_content = f"""<?xml version="1.0" ?>
<sdf version="1.9">
  <world name="random_qr_world">
    <physics name="1ms" type="ignored">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>

    <!-- Map Origin Alignment for PX4 EKF2 Magnetometer/GPS -->
    <spherical_coordinates>
      <surface_model>EARTH_WGS84</surface_model>
      <latitude_deg>47.397742</latitude_deg>
      <longitude_deg>8.545594</longitude_deg>
      <elevation>488.0</elevation>
    </spherical_coordinates>

    <!-- Essential Gazebo Core Plugins -->
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>

    <!-- Required PX4 Sensor Plugins -->
    <plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"/>
    <plugin filename="gz-sim-magnetometer-system" name="gz::sim::systems::Magnetometer"/>
    <plugin filename="gz-sim-air-pressure-system" name="gz::sim::systems::AirPressure"/>
    <plugin filename="gz-sim-navsat-system" name="gz::sim::systems::NavSat"/>
    
    <scene>
      <ambient>1.0 1.0 1.0 1.0</ambient>
      <background>0.8 0.8 0.8 1.0</background>
      <grid>true</grid>
    </scene>

    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 20 0 0 0</pose>
      <diffuse>1.0 1.0 1.0 1</diffuse>
      <specular>0.5 0.5 0.5 1</specular>
      <direction>-0.5 0.1 -0.9</direction>
    </light>

    <!-- Ground Plane -->
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry>
        </collision>
        <visual name="visual">
          <geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry>
          <material>
            <ambient>0.7 0.7 0.7 1</ambient>
            <diffuse>0.7 0.7 0.7 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

    <!-- RED ZONE MAT -->
    <model name="red_zone">
      <static>true</static>
      <pose>{rx} {ry} 0.025 0 0 0</pose>
      <link name="link">
        <visual name="visual">
          <geometry>
            <box><size>{red_w} {red_h} 0.05</size></box>
          </geometry>
          <material>
            <ambient>1.0 0.0 0.0 1.0</ambient>
            <diffuse>1.0 0.0 0.0 1.0</diffuse>
            <specular>0.2 0.0 0.0 1.0</specular>
          </material>
        </visual>
        <collision name="collision">
          <geometry>
            <box><size>{red_w} {red_h} 0.05</size></box>
          </geometry>
        </collision>
      </link>
    </model>
"""

# Inject 5 unique QR Code targets into SDF
for qr in qrs:
    sdf_content += f"""
    <model name="qr_code_{qr['id']}">
      <static>true</static>
      <pose>{qr['x']} {qr['y']} 0.01 0 0 0</pose>
      <link name="link">
        <visual name="visual">
          <geometry>
            <box><size>0.5 0.5 0.01</size></box>
          </geometry>
          <material>
            <ambient>1 1 1 1</ambient>
            <diffuse>1 1 1 1</diffuse>
            <pbr>
              <metal>
                <albedo_map>{qr['image_path']}</albedo_map>
              </metal>
            </pbr>
          </material>
        </visual>
        <collision name="collision">
          <geometry>
            <box><size>0.5 0.5 0.01</size></box>
          </geometry>
        </collision>
      </link>
    </model>"""

sdf_content += "\n  </world>\n</sdf>"

# Save SDF file
os.makedirs(os.path.dirname(WORLD_FILE), exist_ok=True)
with open(WORLD_FILE, "w") as f:
    f.write(sdf_content)

# Export Bounding Box & Labeled Coordinates JSON for ROS 2 Node
world_data = {
    "area_bounds": {
        "x_min": -AREA_WIDTH / 2.0,
        "x_max": AREA_WIDTH / 2.0,
        "y_min": -AREA_HEIGHT / 2.0,
        "y_max": AREA_HEIGHT / 2.0
    }
}

with open(DATA_FILE, "w") as f:
    json.dump(world_data, f, indent=4)

print(f"[WORLD GEN] Successfully generated world with 5 unique, non-overlapping QR codes (1.jpeg - 5.jpeg).")
