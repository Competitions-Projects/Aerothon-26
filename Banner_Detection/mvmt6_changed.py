import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO
import numpy as np

from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleLocalPosition
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy, qos_profile_sensor_data

class YoloAlignmentNode(Node):
    def __init__(self, model_path='bestgz.onnx'):
        super().__init__('yolo_alignment_node')
        
        # QoS Profiles
        pub_qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        sub_qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Subscribers
        self.subscription = self.create_subscription(
            Image, 
            '/camera', 
            self.image_callback, 
            qos_profile_sensor_data
        )
        self.local_pos_sub = self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_odometry',
            self.position_callback,
            sub_qos_profile
        )

        self.bridge = CvBridge()
        
        self.get_logger().info(f"Loading YOLO ONNX model from: {model_path}")
        self.model = YOLO(model_path, task='detect')
        self.get_logger().info("Model loaded successfully.")

        # Publishers
        self.cmd_pub = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', pub_qos_profile)    
        self.mode_pub = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', pub_qos_profile)
        self.setpoint_pub = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', pub_qos_profile)
        
        self.timer = self.create_timer(0.05, self.timer_callback)  # 20 Hz fast loop
        self.counter = 0

        # Drone State & Odometry
        self.current_z = 0.0
        self.current_y = 0.0
        self.target_altitude = -3.0  # -3.0m in PX4 NED system is 3 meters UP
        
        # Latency & Speed Adjustments
        self.frame_count = 0
        self.frame_skip = 1          # Process every frame to eliminate controller latency
        self.aligned_counter = 0

        # High-Speed Non-Oscillating PD Gains
        self.kp_y = 0.0025           # Primary lateral movement
        self.kd_y = 0.0015           # High derivative acts as a brake to stop overshoots
        self.kp_z = 0.002           # Vertical movement
        self.kd_z = 0.003
        self.kp_yaw = 0.0006         # Soft yaw to assist alignment without orbiting
        self.kd_yaw = 0.0006
        
        self.prev_error_x = 0.0
        self.prev_error_y = 0.0
        self.prev_time = None

        # Velocity Targets
        self.target_vx = 0.0
        self.target_vy = 0.0
        self.target_vz = 0.0       
        self.target_yawspeed = 0.0  

        # State Machine: TAKEOFF -> TRACKING -> APPROACH -> HOVER
        self.flight_phase = "TAKEOFF"
        self.lost_ticks = 0

    def position_callback(self, msg):
        self.current_z = msg.z
        self.current_y = msg.y

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

    def image_callback(self, msg):
        # ALLOW camera to process frames during both TRACKING and APPROACH
        if self.flight_phase not in ["TRACKING", "APPROACH"]:
            return

        self.frame_count += 1
        if self.frame_count % self.frame_skip != 0:
            return

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            height, width, _ = frame.shape
            img_center_x, img_center_y = width / 2.0, height / 2.0

            results = self.model(frame, conf=0.85, verbose=False)
            target_found = False

            if len(results[0].boxes) > 0:
                best_box = max(results[0].boxes, key=lambda b: b.conf[0])
                if best_box.conf[0] > 0.25:
                    target_found = True
                    self.lost_ticks = 0
                    
                    xyxy = best_box.xyxy[0].cpu().numpy()
                    box_cx = (xyxy[0] + xyxy[2]) / 2.0
                    box_cy = (xyxy[1] + xyxy[3]) / 2.0

                    # ---------------------------------------------------------
                    # NEW ARDUCAM LOGIC: Calculate Bounding Box Area Ratio
                    # ---------------------------------------------------------
                    box_width = xyxy[2] - xyxy[0]
                    box_height = xyxy[3] - xyxy[1]
                    area_ratio = (box_width * box_height) / (width * height)

                    # If flying forward, check area to trigger brakes
                    if self.flight_phase == "APPROACH":
                        if area_ratio > 0.50:  # Banner takes up 50% of the screen
                            self.flight_phase = "HOVER"
                            self.target_vx = 0.0
                            self.get_logger().info(f"Target close! Area ratio: {area_ratio:.2f}. HOVERING.")
                    
                    # If aligning, do PD Control math
                    elif self.flight_phase == "TRACKING":
                        raw_error_x = box_cx - img_center_x
                        raw_error_y = box_cy - img_center_y 

                        # 1. TIME DELTA CLAMP
                        now = self.get_clock().now().nanoseconds / 1e9
                        dt = (now - self.prev_time) if self.prev_time is not None else 0.05
                        dt = max(dt, 0.02)
                        self.prev_time = now

                        # 2. DERIVATIVE CALCULATION
                        derivative_x = (raw_error_x - self.prev_error_x) / dt
                        derivative_y = (raw_error_y - self.prev_error_y) / dt
                        derivative_x = max(min(derivative_x, 300.0), -300.0)
                        derivative_y = max(min(derivative_y, 300.0), -300.0)

                        self.prev_error_x = raw_error_x
                        self.prev_error_y = raw_error_y

                        # 3. FAST PD CONTROL
                        raw_vy = -((0.0020 * raw_error_x) + (0.0008 * derivative_x))
                        raw_vz = (0.0025 * raw_error_y) + (0.0010 * derivative_y)
                        raw_yawspeed = (0.0005 * raw_error_x) + (0.0002 * derivative_x)

                        self.target_vx = 0.0
                        self.target_vy = max(min(raw_vy, 0.6), -0.6)
                        self.target_vz = max(min(raw_vz, 0.4), -0.4)
                        self.target_yawspeed = max(min(raw_yawspeed, 0.2), -0.2)

                        # 4. LOCK CONDITION (Triggers visual approach, completely bypassing depth)
                        if abs(raw_error_x) < 20.0 and abs(raw_error_y) < 20.0 and abs(derivative_x) < 40.0:
                            self.aligned_counter += 1
                            if self.aligned_counter >= 10:
                                self.get_logger().info("LOCK CONFIRMED! Starting visual approach.")
                                self.flight_phase = "APPROACH"
                                self.target_vy = 0.0
                                self.target_vz = 0.0
                                self.target_yawspeed = 0.0
                        else:
                            self.aligned_counter = max(0, self.aligned_counter - 1)

            if not target_found and self.flight_phase == "TRACKING":
                self.aligned_counter = 0
                self.lost_ticks += 1
                if self.lost_ticks > 15:
                    self.target_vx = 0.0
                    self.target_vy = 0.0
                    self.target_vz = 0.0 
                    self.target_yawspeed = 0.2

            # Display frame safely
            cv2.imshow("Drone Camera", results[0].plot())
            cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f"Vision error: {e}")

    def timer_callback(self):
        timestamp = int(self.get_clock().now().nanoseconds / 1000)

        # Offboard mode message
        offboard_msg = OffboardControlMode()
        offboard_msg.position = False
        offboard_msg.velocity = True
        offboard_msg.timestamp = timestamp
        self.mode_pub.publish(offboard_msg)

        # Main State Machine
        if self.counter > 10:
            if self.flight_phase == "TAKEOFF":
                self.target_vx = 0.0
                self.target_vy = 0.0
                self.target_yawspeed = 0.0
                self.target_vz = -0.8  # Ascend UP
                
                if self.current_z <= self.target_altitude:
                    self.target_vz = 0.0
                    self.flight_phase = "TRACKING"
                    alt_error = self.target_altitude - self.current_z
                    self.target_vz = max(min(0.5 * alt_error, 0.5), -0.5)
                    self.get_logger().info("Target altitude reached. Initiating 3-DOF Alignment (vx = 0.0)...")

            elif self.flight_phase == "APPROACH":
                # FAST FORWARD APPROACH 
                self.target_vx = 1.0       # Accelerated to 1.0 m/s for competition speed
                self.target_vy = 0.0
                self.target_yawspeed = 0.0

                # Level out altitude to -1.5m 
                alt_error = -1.5 - self.current_z
                self.target_vz = max(min(0.5 * alt_error, 0.5), -0.5)

                # The odometry-based stopping trigger has been deleted.
                # Stopping is now handled dynamically by the Arducam bounding box area.

            elif self.flight_phase == "HOVER":
                self.target_vx = 0.0
                self.target_vy = 0.0
                self.target_yawspeed = 0.0
                
                # Active altitude lock to prevent drifting
                alt_error = -1.5 - self.current_z
                self.target_vz = max(min(0.5 * alt_error, 0.3), -0.3)

        # Publish Setpoint 
        traj_msg = TrajectorySetpoint()
        traj_msg.position = [float('nan'), float('nan'), float('nan')]
        traj_msg.velocity = [float(self.target_vy), float(self.target_vx), float(self.target_vz)]
        traj_msg.yaw = float('nan')
        traj_msg.yawspeed = float(self.target_yawspeed)
        traj_msg.timestamp = timestamp
        self.setpoint_pub.publish(traj_msg)

        # Arming commands
        if self.counter >= 10 and self.counter % 10 == 0:
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
                
        self.counter += 1

def main(args=None):
    rclpy.init(args=args)
    node = YoloAlignmentNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()