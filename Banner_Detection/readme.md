🟢 Green Banner Detection

Vision and flight scripts for the autonomous banner-tracking and corridor-entry sequence.
📁 Files
* **`mvmt6_arducam.py`:** Flight version. Since the depth camera is delayed, this estimates distance using the YOLO bounding box size from the standard Arducam. Runs headless.
* **`mvmt6_changed.py`:** Michael's updated code. Fixed to use real hardware topics (odometry/depth) instead of Gazebo simu topics. Includes a live video feed for monitor debugging.
🚁 Flight Sequence
1. **Takeoff:** Ascends and looks for the green banner.
2. **Align:** Adjusts position to perfectly center the banner in the camera.
3. **Descend:** Drops 3 meters to line up with the corridor entrance.
4. **Approach:** Moves straight forward to enter the corridor.
5. ** Hover: ** Brakes and holds position at the target distance.

*Note: YOLO models (`best.onnx`, `bestgz.onnx`) are too large for GitHub. Download them from the team Drive and place them in the root folder before flying.*
