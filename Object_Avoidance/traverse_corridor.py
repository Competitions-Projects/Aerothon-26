#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import Image
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleLocalPosition,VehicleAttitude
import numpy as np
from rclpy.qos import qos_profile_sensor_data
import math


class ObstacleAvoidance(Node):
    def __init__(self):
        super().__init__('obstacle_avoidance')

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # PX4 command publishers (same as before)
        self.offboard_pub = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.trajectory_pub = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        self.command_pub = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos_profile)
        self.local_pos_sub = self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position_v1',
            self.position_callback, qos_profile)
        self.attitude_sub = self.create_subscription(
            VehicleAttitude, '/fmu/out/vehicle_attitude',
            self.attitude_callback, qos_profile)

        # Depth camera subscriber — update this topic to match your bridge
        self.depth_sub = self.create_subscription(
            Image,
            '/depth_camera',
            self.depth_callback,
            qos_profile_sensor_data
        )

        self.phase = 'climb'

        self.t_vx = 0.0
        self.t_vy = 0.0
        self.t_vz = 0.0

        self.t_yaw = 0.0

        self.cruise_vel_z = 1
        self.current_yaw = 0.0


        self.current_z = 0.0
        self.min_front_distance = float('inf')
        self.setpoint_counter = 0

        self.SAFE_DISTANCE = 1.0   # meters — stop/avoid if something closer than this
        self.safe_hold_dist = 1
        self.CRUISE_VELOCITY = 1.0  # m/s forward speed when clear
        self.TARGET_ALTITUDE = -3.0  # NED, 3m up

        self.timer = self.create_timer(0.1, self.control_loop)

    def position_callback(self, msg):
        self.current_z = msg.z
    
    def attitude_callback(self, msg):
        q = msg.q  # [w, x, y, z]
        siny_cosp = 2.0 * (q[0] * q[3] + q[1] * q[2])
        cosy_cosp = 1.0 - 2.0 * (q[2] * q[2] + q[3] * q[3])
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)

    def depth_callback(self, msg):
        # Depth images are typically 32-bit float, meters per pixel
        depth_array = np.frombuffer(msg.data, dtype=np.float32).reshape(msg.height, msg.width)

        # Look at a center region only (avoid floor/ceiling/edges skewing the reading)
        h, w = depth_array.shape
        center_region = depth_array[h//3 : 2*h//3, w//3 : 2*w//3]

        # Filter out invalid readings (inf/nan are common at max range or no return)
        valid = center_region[np.isfinite(center_region)]
        if len(valid) > 0:
            self.min_front_distance = float(np.min(valid))
        else:
            self.min_front_distance = float('inf')

    def publish_offboard_mode(self):
        msg = OffboardControlMode()
        msg.position = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_pub.publish(msg)

    def publish_velocity_setpoint(self, vx, vy, vz, yaw=None):
        msg = TrajectorySetpoint()
        msg.velocity = [vx, vy, vz]
        msg.position = [float('nan'), float('nan'), float('nan')]
        if yaw is not None:
            msg.yaw = yaw
        else:
            msg.yaw = float('nan')  # nan = "don't care, keep current heading"
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_pub.publish(msg)

    

    def set_velocity_setpoint(self, vx, vy, vz):
        self.t_vx = vx
        self.t_vy = vy
        self.t_vz = vz

    def set_target_height(self,th):
        v = self.cruise_vel_z*(1+(self.current_z/th))

    def set_rel_vel(self,vx,vy,vz,yaw=None):

        if(yaw is not None):
            self.t_yaw = yaw
        else:
            self.t_yaw = current_yaw

        self.t_vx = vx*math.cos(self.t_yaw)-vy*math.sin(self.t_yaw)
        self.t_vy = vx*math.sin(self.t_yaw) + vy*math.cos(self.t_yaw)
        self.t_vz = vz
        




    def publish_vehicle_command(self, command, param1=0.0, param2=0.0):
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = param1
        msg.param2 = param2
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.command_pub.publish(msg)

    def control_loop(self):
        self.publish_offboard_mode()
        self.set_velocity_setpoint(0,0,0)

        

        if self.setpoint_counter == 10:
            self.publish_vehicle_command(176, param1=1.0, param2=6.0)
            self.publish_vehicle_command(400, param1=1.0)

        current_altitude = -self.current_z
        yaw = 1.57

        ctime = 6*10

        self.get_logger().info(f'Alt:{current_altitude}, phase: {self.phase}, dist:{self.min_front_distance:.2f},yaw:{self.current_yaw:.2f}')

        if self.phase == 'climb':
            # climb straight up to cruising altitude first
            self.set_velocity_setpoint(0.0, 0.0, -0.5)  # NED: negative vz = upward
            if current_altitude > 2.8:  # close enough to 3m target
                self.get_logger().info('Reached cruising altitude, starting forward motion')
                self.phase = 'test'

        elif self.phase == 'cruise':
            if self.min_front_distance < self.SAFE_DISTANCE:
                self.get_logger().info(f'Obstacle at {self.min_front_distance:.2f}m — stopping')
                self.set_velocity_setpoint(-self.CRUISE_VELOCITY*(1-(self.min_front_distance/self.safe_hold_dist)), 0.0, 0.0)
            else:
                self.set_velocity_setpoint(self.CRUISE_VELOCITY, 0.0, 0.0)
        elif self.phase == 'avoid':
            if self.min_front_distance < self.SAFE_DISTANCE:
                self.get_logger().info(f'Obstacle at {self.min_front_distance:.2f}m — stopping')
                self.set_velocity_setpoint(-self.CRUISE_VELOCITY*(1-(self.min_front_distance/self.safe_hold_dist)), 0.0, 0.0)
            else:
                self.set_velocity_setpoint(0.0, 0.0, 0.0)
        elif self.phase == 'test':
            if(self.setpoint_counter%150 > 75):
                yaw = 2.35
            else:
                yaw = 0.785
            self.set_rel_vel(-self.CRUISE_VELOCITY*(1-(self.min_front_distance/self.safe_hold_dist)),0.0,0.0,yaw)
        

        self.publish_velocity_setpoint(self.t_vx,self.t_vy,self.t_vz,self.t_yaw)

        self.setpoint_counter += 1

def main():
    rclpy.init()
    node = ObstacleAvoidance()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()