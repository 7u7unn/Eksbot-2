#!/usr/bin/env python3


import rclpy
import math
import time
from rclpy.node import Node
from rclpy.parameter import Parameter
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

class WaypointNavigator(Node):
    def __init__(self):
        super().__init__('waypoint_navigation')


        self.declare_parameter('waypoint_1_x', 2.0)
        self.declare_parameter('waypoint_1_y', 1.0)
        self.declare_parameter('waypoint_2_x', 4.0)
        self.declare_parameter('waypoint_2_y', 3.0)
        self.declare_parameter('kp', 1.0)
        self.declare_parameter('ki', 0.0)
        self.declare_parameter('kd', 0.1)

        self.waypoints = [
            (self.get_parameter('waypoint_1_x').value, self.get_parameter('waypoint_1_y').value),
            (self.get_parameter('waypoint_2_x').value, self.get_parameter('waypoint_2_y').value)
        ]
        self.kp = self.get_parameter('kp').value
        self.ki = self.get_parameter('ki').value
        self.kd = self.get_parameter('kd').value

        # ROS 2 subscriptions and publishers
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # PID Controller Variables
        self.prev_error = 0.0
        self.integral = 0.0
        self.last_time = time.time()
        
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_theta = 0.0
        self.current_waypoint_idx = 0

    def odom_callback(self, msg):

        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y


        orientation = msg.pose.pose.orientation
        _, _, self.current_theta = self.quaternion_to_euler(orientation.x, orientation.y, orientation.z, orientation.w)

        # Check if the robot reached the waypoint
        if self.current_waypoint_idx < len(self.waypoints):
            target_x, target_y = self.waypoints[self.current_waypoint_idx]
            distance = math.sqrt((target_x - self.current_x) ** 2 + (target_y - self.current_y) ** 2)

            if distance < 0.2:  # Threshold to consider waypoint reached
                self.get_logger().info(f"Reached waypoint {self.current_waypoint_idx + 1}")
                self.current_waypoint_idx += 1

                if self.current_waypoint_idx >= len(self.waypoints):
                    self.get_logger().info("All waypoints reached. Stopping.")
                    self.stop_robot()
                    return

            self.navigate_to_waypoint(target_x, target_y)

    def navigate_to_waypoint(self, target_x, target_y):

        error_x = target_x - self.current_x
        error_y = target_y - self.current_y
        angle_to_target = math.atan2(error_y, error_x)

        # PID Control
        angular_error = angle_to_target - self.current_theta
        angular_error = math.atan2(math.sin(angular_error), math.cos(angular_error))  # Normalize to [-pi, pi]

        self.integral += angular_error
        derivative = (angular_error - self.prev_error) / max((time.time() - self.last_time), 0.001)
        self.last_time = time.time()

        angular_speed = self.kp * angular_error + self.ki * self.integral + self.kd * derivative
        linear_speed = 0.2


        cmd = Twist()
        cmd.linear.x = linear_speed
        cmd.angular.z = angular_speed
        self.cmd_vel_pub.publish(cmd)

        self.prev_error = angular_error

    def stop_robot(self):
        """Stop the robot after reaching the final waypoint."""
        cmd = Twist()
        self.cmd_vel_pub.publish(cmd)

    @staticmethod
    def quaternion_to_euler(x, y, z, w):
        """Convert quaternion to roll, pitch, yaw (Euler angles)."""
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)

        return roll_x, pitch_y, yaw_z

def main(args=None):
    rclpy.init(args=args)
    navigator = WaypointNavigator()
    rclpy.spin(navigator)
    navigator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()