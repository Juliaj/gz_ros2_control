#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
import math
import time
import sys
import argparse

class AckermannDriveExampleTestHelper(Node):
    def __init__(self, initial_mode=0, auto_cycle=True, duration=5.0):
        super().__init__('ackermann_drive_example_test_helper')
        
        # Create publisher for the reference topic used by the Ackermann controller
        self.publisher = self.create_publisher(
            TwistStamped, 
            '/ackermann_steering_controller/reference', 
            10
        )
        
        # Subscribe to odometry from Gazebo
        self.odom_subscription = self.create_subscription(
            Odometry,
            '/ackermann_steering_controller/odom',  # Adjust topic name if needed
            self.odom_callback,
            10
        )
        
        # Store latest odometry data
        self.current_odom = None
        self.commanded_vel = None
        
        # Timer for publishing commands
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        # Parameters for circular motion
        self.linear_speed = 0.3  # m/s
        self.angular_speed = 0.2  # rad/s - positive for counterclockwise
        
        # Testing mode and timing
        self.test_mode = initial_mode  # 0: normal, 1: rapid acceleration, 2: sharp turns, 3: direction changes
        self.auto_cycle = auto_cycle   # Whether to automatically cycle between modes
        self.mode_start_time = self.get_clock().now()
        self.mode_duration = duration  # seconds per test mode
        
        # Mode descriptions for logging
        self.mode_descriptions = [
            "Normal circular motion",
            "Rapid acceleration (wheel slip test)",
            "Sharp turns at high speed (wheel slip test)",
            "Rapid direction changes (wheel slip test)"
        ]
        
        self.get_logger().info(f'Starting with test mode {self.test_mode}: {self.mode_descriptions[self.test_mode]}')
        self.get_logger().info(f'Auto-cycle between modes: {self.auto_cycle}')
        
        # For slip detection
        self.slip_detection_timer = self.create_timer(1.0, self.check_for_slip)

    def odom_callback(self, msg):
        """Store the latest odometry data from Gazebo"""
        self.current_odom = msg
        # Log position and orientation from odometry
        pos_x = self.current_odom.pose.pose.position.x
        pos_y = self.current_odom.pose.pose.position.y
        
        self.get_logger().info(
                f'Slip Analysis - Mode {self.test_mode}:\n'
                f'  Position: X={pos_x:.2f}, Y={pos_y:.2f}\n'
            )
    
    def check_for_slip(self):
        """Compare commanded velocity with actual odometry to detect slip"""
        if self.current_odom is None or self.commanded_vel is None:
            return
            
        # Extract actual velocities from odometry
        actual_linear_x = self.current_odom.twist.twist.linear.x
        actual_angular_z = self.current_odom.twist.twist.angular.z
        
        # Compare with commanded velocities
        if self.commanded_vel is not None:
            linear_diff = abs(self.commanded_vel.linear.x - actual_linear_x)
            angular_diff = abs(self.commanded_vel.angular.z - actual_angular_z)
            
            # Calculate error percentage
            linear_error_pct = 100.0 * linear_diff / max(0.01, abs(self.commanded_vel.linear.x))
            angular_error_pct = 100.0 * angular_diff / max(0.01, abs(self.commanded_vel.angular.z))
            
            # Log position and orientation from odometry
            pos_x = self.current_odom.pose.pose.position.x
            pos_y = self.current_odom.pose.pose.position.y
            
            self.get_logger().info(
                f'Slip Analysis - Mode {self.test_mode}:\n'
                f'  Position: X={pos_x:.2f}, Y={pos_y:.2f}\n'
                f'  CMD vs Actual: Linear={self.commanded_vel.linear.x:.2f} vs {actual_linear_x:.2f} ({linear_error_pct:.1f}% diff)\n'
                f'  CMD vs Actual: Angular={self.commanded_vel.angular.z:.2f} vs {actual_angular_z:.2f} ({angular_error_pct:.1f}% diff)'
            )
            
            # Detect significant slip
            if linear_error_pct > 20.0 or angular_error_pct > 20.0:
                self.get_logger().warning(f'WHEEL SLIP DETECTED in mode {self.test_mode}')

    def set_test_mode(self, mode):
        """Manually set the test mode (0-3)"""
        if 0 <= mode <= 3:
            self.test_mode = mode
            self.mode_start_time = self.get_clock().now()
            self.get_logger().info(f'Manually switched to test mode {self.test_mode}: {self.mode_descriptions[self.test_mode]}')
        else:
            self.get_logger().error(f'Invalid test mode: {mode}. Must be between 0-3.')

    def timer_callback(self):
        current_time = self.get_clock().now()
        elapsed = (current_time - self.mode_start_time).nanoseconds / 1e9
        
        # Switch test modes every mode_duration seconds if auto-cycle is enabled
        if self.auto_cycle and elapsed > self.mode_duration:
            self.test_mode = (self.test_mode + 1) % 4
            self.mode_start_time = current_time
            self.get_logger().info(f'Switching to test mode {self.test_mode}: {self.mode_descriptions[self.test_mode]}')
        
        # Create TwistStamped message
        msg = TwistStamped()
        msg.header.stamp = current_time.to_msg()
        msg.header.frame_id = "base_link"
        
        # Set velocities based on test mode
        if self.test_mode == 0:
            # Normal circular motion
            msg.twist.linear.x = self.linear_speed
            msg.twist.angular.z = self.angular_speed
        elif self.test_mode == 1:
            # Rapid acceleration - sudden increase in linear velocity
            progress = elapsed / self.mode_duration
            if progress < 0.5:
                msg.twist.linear.x = self.linear_speed
            else:
                msg.twist.linear.x = self.linear_speed * 3.0  # Triple the speed suddenly
            msg.twist.angular.z = self.angular_speed
        elif self.test_mode == 2:
            # Sharp turns at high speed - cause slippage
            msg.twist.linear.x = self.linear_speed * 2.0
            # Oscillating angular velocity for sharp turns
            msg.twist.angular.z = self.angular_speed * 3.0 * math.sin(elapsed * 4.0)
        elif self.test_mode == 3:
            # Rapid direction changes - alternate forward/reverse
            msg.twist.linear.x = self.linear_speed * 2.0 * math.sin(elapsed * 3.0)
            msg.twist.angular.z = self.angular_speed
        
        # Zero out other components
        msg.twist.linear.y = 0.0
        msg.twist.linear.z = 0.0
        msg.twist.angular.x = 0.0
        msg.twist.angular.y = 0.0
        
        # Store commanded velocity for slip detection
        self.commanded_vel = msg.twist
        
        # Publish the message
        self.publisher.publish(msg)
        # self.get_logger().info(f'Mode {self.test_mode}: Linear={msg.twist.linear.x:.2f}, Angular={msg.twist.angular.z:.2f}')

def main(args=None):
    # Parse command line arguments
    parser = argparse.ArgumentParser(description='Ackermann Drive Test Helper with wheel slip testing')
    parser.add_argument('--mode', type=int, default=1, choices=[0, 1, 2, 3],
                        help='Test mode: 0=normal, 1=rapid acceleration, 2=sharp turns, 3=direction changes')
    parser.add_argument('--no-cycle', action='store_false',
                        help='Disable automatic cycling between test modes')
    parser.add_argument('--duration', type=float, default=1.0,
                        help='Duration of each test mode in seconds (when auto-cycling)')
    parser.add_argument('--odom-topic', type=str, default='/ackermann_steering_controller/odom',
                        help='Topic name for odometry data')
    
    # Parse args before ROS initialization
    if args is None:
        args = sys.argv[1:]
    parsed_args = parser.parse_args(args)
    
    rclpy.init(args=args)
    node = AckermannDriveExampleTestHelper(
        initial_mode=parsed_args.mode,
        auto_cycle=not parsed_args.no_cycle,
        duration=parsed_args.duration
    )
    
    # Update odometry topic if provided via command line
    if parsed_args.odom_topic:
        node.odom_subscription = node.create_subscription(
            Odometry,
            parsed_args.odom_topic,
            node.odom_callback,
            10
        )
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt, shutting down')
    finally:
        # Stop the robot before shutting down
        stop_msg = TwistStamped()
        stop_msg.header.stamp = node.get_clock().now().to_msg()
        stop_msg.header.frame_id = "base_link"
        node.publisher.publish(stop_msg)
        node.get_logger().info('Robot stopped')
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()


# test command:
#    python3 ackermann_drive_example_test_helper.py --mode 1 --no-cycle
#       python3 ackermann_drive_example_test_helper.py --mode 2
#    python3 ackermann_drive_example_test_helper.py --mode 3 --no-cycle
#    python3 ackermann_drive_example_test_helper.py --duration 10