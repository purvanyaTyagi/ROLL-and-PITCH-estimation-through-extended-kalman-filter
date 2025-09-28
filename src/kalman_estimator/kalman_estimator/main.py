import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Quaternion
import math
import numpy as np

class ImuVisualizer(Node):
    def __init__(self):
        super().__init__('imu_visualizer')
        self.roll = 0.0
        self.pitch = 0.0

        self.last_time = self.get_clock().now()

        self.sub = self.create_subscription(
            Imu,
            '/imu/data',
            self.imu_callback,
            10
        )

        self.roll_gyro_raw = 0.0
        self.roll_est = 0.0
        self.roll_var = 1.0

        self.pitch_gyro_raw = 0.0
        self.pitch_est = 0.0
        self.pitch_var = 1.0

        self.sigma_w = 0.01
        self.sigma_v = 1.0

        self.marker_pub = self.create_publisher(Marker, 'imu_marker', 10)
        self.marker_pub_raw_integral = self.create_publisher(Marker, 'imu_marker_integration_only', 10)


    def imu_callback(self, msg):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        if dt == 0.0:
            return
        self.last_time = now

        gyro_x = msg.angular_velocity.x
        gyro_y = msg.angular_velocity.y

        self.roll_gyro_raw += dt * gyro_x * 100
        self.pitch_gyro_raw += dt * gyro_y * 100

        acc_roll = math.atan2(
            msg.linear_acceleration.y,
            math.sqrt(msg.linear_acceleration.x * msg.linear_acceleration.x + msg.linear_acceleration.z * msg.linear_acceleration.z)
        ) 

        acc_pitch = math.atan2(
            -msg.linear_acceleration.x,
            math.sqrt(msg.linear_acceleration.y * msg.linear_acceleration.y +msg.linear_acceleration.z * msg.linear_acceleration.z)
        )

        roll_pred = self.roll_est + dt*gyro_x
        roll_var_pred = self.roll_var + dt * dt * self.sigma_w;

        pitch_pred = self.pitch_est + dt*gyro_y
        pitch_var_pred = self.pitch_var + dt * dt * self.sigma_w;


        K_roll = roll_var_pred / (roll_var_pred + dt * dt * self.sigma_v)  # Kalman gain
        K_pitch = pitch_var_pred / (pitch_var_pred + dt * dt * self.sigma_v)
        self.roll_est = roll_pred + K_roll * (acc_roll - roll_pred)    # Update roll
        self.pitch_est = pitch_pred + K_pitch * (acc_pitch - pitch_pred)    # Update pitch
        self.roll_var = (1 - K_roll) * roll_var_pred                        # Update variance
        self.pitch_var = (1 - K_pitch) * pitch_var_pred                        
        # ---------------- RVIZ MARKER ---------------------
        cy = math.cos(0.0 * 0.5)
        sy = math.sin(0.0 * 0.5)
        cp = math.cos(self.pitch_est * 0.5)
        sp = math.sin(self.pitch_est * 0.5)
        cr = math.cos(self.roll_est * 0.5)
        sr = math.sin(self.roll_est * 0.5)

        q = Quaternion()
        q.w = cr * cp * cy + sr * sp * sy
        q.x = sr * cp * cy - cr * sp * sy
        q.y = cr * sp * cy + sr * cp * sy
        q.z = cr * cp * sy - sr * sp * cy

        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = now.to_msg()
        marker.ns = 'imu'
        marker.id = 0
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose.orientation = q
        marker.pose.position.x = -2.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.0
        marker.scale.x = 1.0
        marker.scale.y = 1.0
        marker.scale.z = 0.1
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        self.marker_pub.publish(marker)

        #------------------------------------------
        cy = math.cos(0.0 * 0.5)
        sy = math.sin(0.0 * 0.5)
        cp = math.cos(self.pitch_gyro_raw * 0.5)
        sp = math.sin(self.pitch_gyro_raw * 0.5)
        cr = math.cos(self.roll_gyro_raw * 0.5)
        sr = math.sin(self.roll_gyro_raw * 0.5)

        q = Quaternion()
        q.w = cr * cp * cy + sr * sp * sy
        q.x = sr * cp * cy - cr * sp * sy
        q.y = cr * sp * cy + sr * cp * sy
        q.z = cr * cp * sy - sr * sp * cy

        marker_raw = Marker()
        marker_raw.header.frame_id = 'map'
        marker_raw.header.stamp = now.to_msg()
        marker_raw.ns = 'imu'
        marker_raw.id = 0
        marker_raw.type = Marker.CUBE
        marker_raw.action = Marker.ADD
        marker_raw.pose.orientation = q
        marker_raw.pose.position.x = 2.0
        marker_raw.pose.position.y = 0.0
        marker_raw.pose.position.z = 0.0
        marker_raw.scale.x = 1.0
        marker_raw.scale.y = 1.0
        marker_raw.scale.z = 0.1
        marker_raw.color.r = 0.0
        marker_raw.color.g = 0.0
        marker_raw.color.b = 1.0
        marker_raw.color.a = 1.0

        self.marker_pub_raw_integral.publish(marker_raw)


def main():
    rclpy.init()
    node = ImuVisualizer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
