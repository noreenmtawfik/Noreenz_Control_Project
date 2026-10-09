"""
Teleoperation bridge node:
Subscribes to standard geometry_msgs/Twist on /cmd_vel (from teleop_twist_keyboard or joy)
and translates it to /throttle (Float32 in [-1.0, 1.0]) and /steer (Float32 in radians).

Supports two progression phases:
- Phase 1 (Milestone 3): Open-loop feedforward mapping with a safety watchdog timer.
- Phase 2 (Milestone 4): Closed-loop speed regulation using PIDLongitudinalController.
"""

import numpy as np  # noqa: F401
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32

# ==============================================================================
# Phase 2 (Milestone 4): Uncomment these imports when upgrading to cruise control
# ==============================================================================
from nav_msgs.msg import Odometry
from bicycle_control.longitudinal_pid import PIDLongitudinalController


class TeleopBridge(Node):
    def __init__(self):
        super().__init__('teleop_bridge')
        self.get_logger().info('Teleoperation Bridge Node Initialized')

        # Parameters
        self.declare_parameter('max_linear_vel', 5.0)     # m/s corresponding to full 1.0 throttle
        self.declare_parameter('max_angular_vel', 1.0)    # rad/s corresponding to full steering
        self.declare_parameter('max_steer_rad', 0.610865)  # radians (~35 degrees)
        self.declare_parameter('auto_zero_timeout', 0.5)  # seconds before zeroing commands
        self.declare_parameter('use_cruise_control', False)  # Enable in Milestone 4.2

        self.max_linear_vel = float(self.get_parameter('max_linear_vel').value)
        self.max_angular_vel = float(self.get_parameter('max_angular_vel').value)
        self.max_steer_rad = float(self.get_parameter('max_steer_rad').value)
        self.auto_zero_timeout = float(self.get_parameter('auto_zero_timeout').value)
        self.use_cruise_control = bool(self.get_parameter('use_cruise_control').value)

        # Publishers (10 Hz rate per assignment specification)
        self.throttle_pub = self.create_publisher(Float32, '/throttle', 10)
        self.steer_pub = self.create_publisher(Float32, '/steer', 10)

        # Subscribers
        self.cmd_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_callback, 10)

        self.current_throttle = 0.0
        self.current_steer = 0.0
        self.target_vel = 0.0
        self.last_cmd_time = self.get_clock().now()

        # TODO: Phase 2 (Milestone 4.2) — Closed-Loop Cruise Control Setup
        # This allows the car to automatically hold a steady speed instead of requiring manual throttle.
        # Initialize the PID speed controller and subscribe to odometry data.
        self.current_vel = 0.0
        self.pid_speed = PIDLongitudinalController(dt=0.1)
        self.odom_sub = self.create_subscription(Odometry, '/state', self.odom_callback, 10)

        # Publish loop at 10 Hz
        self.timer = self.create_timer(0.1, self.publish_commands)

    def odom_callback(self, msg: Odometry):
        """Milestone 4.2: Extracts vehicle forward speed from /state odometry."""
        self.current_vel = float(msg.twist.twist.linear.x)

    def cmd_callback(self, msg: Twist):
        """Translates Twist linear.x to throttle [-1, 1] and angular.z into steering (rad)."""
        # TODO: Milestone 3.1 — Teleoperation Command Mapping
        # This connects user inputs (keyboard/joystick) to the car's physical actuators.
        # Map the incoming Twist linear/angular commands to throttle and steering.
        
        self.last_cmd_time = self.get_clock().now()
        self.target_vel = float(msg.linear.x)

        # Milestone 3.1: Open-loop throttle mapping
        if self.max_linear_vel > 0.0:
            throttle = msg.linear.x / self.max_linear_vel
        else:
            throttle = 0.0
        self.current_throttle = float(np.clip(throttle, -1.0, 1.0))

        # Milestone 3.1: Steering angle mapping from angular velocity
        if self.max_angular_vel > 0.0:
            steer = (msg.angular.z / self.max_angular_vel) * self.max_steer_rad
        else:
            steer = 0.0
        self.current_steer = float(np.clip(steer, -self.max_steer_rad, self.max_steer_rad))

    def publish_commands(self):
        """Periodically publishes throttle and steering commands at 10 Hz with safety watchdog."""
        time_diff = (self.get_clock().now() - self.last_cmd_time).nanoseconds * 1e-9

        if time_diff > self.auto_zero_timeout:
            throttle_to_send = 0.0
            steer_to_send = 0.0
            if self.use_cruise_control:
                self.pid_speed.reset()
        else:
            steer_to_send = self.current_steer
            if self.use_cruise_control:
                # Closed-loop speed control via PID
                throttle_to_send = self.pid_speed.compute(self.target_vel, self.current_vel)
            else:
                throttle_to_send = self.current_throttle

        throttle_msg = Float32()
        throttle_msg.data = float(throttle_to_send)
        self.throttle_pub.publish(throttle_msg)

        steer_msg = Float32()
        steer_msg.data = float(steer_to_send)
        self.steer_pub.publish(steer_msg)


def main(args=None):
    rclpy.init(args=args)
    bridge = TeleopBridge()
    try:
        rclpy.spin(bridge)
    except KeyboardInterrupt:
        pass
    finally:
        bridge.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
