#!/usr/bin/env python3
"""
Enhanced Velocity Bridge Node for Caterpillar Rover (Nav2 Compatible)
Bridges between Nav2's geometry_msgs/Twist and the custom Velocity message
with autonomous crater avoidance capability.

Subscribes:
    /cmd_vel (geometry_msgs/Twist) - Standard Nav2 velocity commands
    /mode_switch (robot_interfaces/ModeSwitch) - .autonomous field for mode
    /ml_pipeline (std_msgs/String) - ML crater detection output
    
Publishes:
    /velocity (robot_interfaces/Velocity) - Custom message for Blitz/MCU
    
ML Pipeline Format:
    "NO_CRATER" - No crater detected, continue normal operation
    "CRATER x1=59 y1=407 x2=336 y2=569 conf=0.899" - Crater detected with coordinates
    
Architecture:
    Nav2 -> /cmd_vel -> [Velocity Bridge] -> /velocity -> MCU
    
    When crater detected:
    - Velocity Bridge executes hardcoded avoidance maneuver
    - Nav2 continues planning in background (NOT shutdown)
    - After avoidance complete, Nav2's cmd_vel resumes control
    - Nav2 automatically continues to goal without interruption
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from robot_interfaces.msg import Velocity, ModeSwitch
from enum import Enum
import time
import math
import re

class AvoidanceState(Enum):
    NORMAL = 0
    MOVE_BACK = 1
    TURN_RIGHT = 2
    MOVE_RIGHT = 3
    TURN_LEFT = 4
    MOVE_FORWARD = 5
    COMPLETE = 6

class VelocityBridge(Node):
    def __init__(self):
        super().__init__('velocity_bridge')
        
        # Parameters
        self.declare_parameter('input_topic', '/cmd_vel')
        self.declare_parameter('output_topic', '/velocity')
        self.declare_parameter('max_linear_vel', 1.0)   # m/s
        self.declare_parameter('max_angular_vel', 2.0)  # rad/s
        self.declare_parameter('avoidance_linear_vel', 0.3)  # m/s for avoidance
        self.declare_parameter('avoidance_angular_vel', 0.5)  # rad/s for turns
        self.declare_parameter('backward_distance', 0.2)  # meters to move back
        self.declare_parameter('pixel_to_meter_ratio', 0.01)  # Convert pixels to meters
        self.declare_parameter('min_confidence', 0.7)  # Minimum confidence threshold
        
        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        self.max_linear = self.get_parameter('max_linear_vel').value
        self.max_angular = self.get_parameter('max_angular_vel').value
        self.avoid_linear = self.get_parameter('avoidance_linear_vel').value
        self.avoid_angular = self.get_parameter('avoidance_angular_vel').value
        self.backward_dist = self.get_parameter('backward_distance').value
        self.pixel_to_meter = self.get_parameter('pixel_to_meter_ratio').value
        self.min_confidence = self.get_parameter('min_confidence').value
        
        # State variables
        self.is_autonomous = False
        self.avoidance_state = AvoidanceState.NORMAL
        self.crater_width = 0.0  # in meters
        self.crater_length = 0.0  # in meters
        self.maneuver_start_time = 0.0
        self.current_maneuver_duration = 0.0
        self.last_cmd_vel = Twist()
        
        # Subscriber to Nav2's cmd_vel
        self.twist_sub = self.create_subscription(
            Twist, input_topic, self.twist_callback, 10)
        
        # Subscriber to mode switch (using ModeSwitch message)
        self.mode_sub = self.create_subscription(
            ModeSwitch, '/mode_switch', self.mode_callback, 10)
        
        # Subscriber to ML pipeline
        self.ml_sub = self.create_subscription(
            String, '/ml_pipeline', self.ml_callback, 10)
        
        # Publisher to Blitz/MCU
        self.vel_pub = self.create_publisher(Velocity, output_topic, 10)
        
        # Timer for avoidance state machine (100Hz for smooth control)
        self.avoidance_timer = self.create_timer(0.01, self.avoidance_state_machine)
        
        self.get_logger().info('='*60)
        self.get_logger().info(f'Velocity Bridge Node Started')
        self.get_logger().info(f'  Input:  {input_topic} (from Nav2)')
        self.get_logger().info(f'  Output: {output_topic} (to MCU/Blitz)')
        self.get_logger().info(f'  Mode:   Waiting for /mode_switch...')
        self.get_logger().info(f'  Crater Avoidance: ENABLED')
        self.get_logger().info(f'  Pixel to Meter: {self.pixel_to_meter:.4f}')
        self.get_logger().info(f'  Min Confidence: {self.min_confidence:.2f}')
        self.get_logger().info('='*60)
    
    def mode_callback(self, msg: ModeSwitch):
        """Handle mode switch changes using ModeSwitch message."""
        if self.is_autonomous != msg.autonomous:
            self.is_autonomous = msg.autonomous
            mode_str = "AUTONOMOUS" if self.is_autonomous else "MANUAL"
            self.get_logger().info(f'MODE SWITCHED: {mode_str}')
            
            # Reset avoidance state when switching to manual
            if not self.is_autonomous and self.avoidance_state != AvoidanceState.NORMAL:
                self.get_logger().warn('Switching to MANUAL - Aborting crater avoidance')
                self.avoidance_state = AvoidanceState.NORMAL
                # Send stop command
                self.publish_stop()
    
    def parse_crater_detection(self, data: str) -> dict:
        """
        Parse crater detection string.
        Format: "CRATER x1=59 y1=407 x2=336 y2=569 conf=0.899"
        Returns: dict with x1, y1, x2, y2, conf or None if invalid
        """
        # Extract values using regex
        pattern = r'CRATER\s+x1=(\d+)\s+y1=(\d+)\s+x2=(\d+)\s+y2=(\d+)\s+conf=([\d.]+)'
        match = re.match(pattern, data)
        
        if not match:
            return None
        
        try:
            x1 = int(match.group(1))
            y1 = int(match.group(2))
            x2 = int(match.group(3))
            y2 = int(match.group(4))
            conf = float(match.group(5))
            
            return {
                'x1': x1,
                'y1': y1,
                'x2': x2,
                'y2': y2,
                'confidence': conf
            }
        except (ValueError, IndexError) as e:
            self.get_logger().error(f'Failed to parse crater values: {e}')
            return None
    
    def ml_callback(self, msg: String):
        """Handle ML pipeline crater detection."""
        # Only process in autonomous mode and when not already avoiding
        if not self.is_autonomous:
            return
            
        if self.avoidance_state != AvoidanceState.NORMAL:
            # Already executing avoidance - ignore new detections
            return
        
        data = msg.data.strip()
        
        if data == "NO_CRATER":
            # Continue normal operation - cmd_vel passes through
            return
        
        elif data.startswith("CRATER"):
            # Parse crater detection
            crater_info = self.parse_crater_detection(data)
            
            if crater_info is None:
                self.get_logger().error(f'Invalid CRATER format: {data}')
                return
            
            # Check confidence threshold
            if crater_info['confidence'] < self.min_confidence:
                self.get_logger().warn(
                    f'Crater confidence {crater_info["confidence"]:.3f} below threshold '
                    f'{self.min_confidence:.2f}, ignoring'
                )
                return
            
            # Calculate crater dimensions in pixels
            width_pixels = abs(crater_info['x2'] - crater_info['x1'])
            height_pixels = abs(crater_info['y2'] - crater_info['y1'])
            
            # Convert to meters
            self.crater_width = width_pixels * self.pixel_to_meter
            self.crater_length = height_pixels * self.pixel_to_meter
            
            self.get_logger().warn('='*60)
            self.get_logger().warn('CRATER DETECTED!')
            self.get_logger().warn(f'  Coordinates: x1={crater_info["x1"]}, y1={crater_info["y1"]}, '
                                  f'x2={crater_info["x2"]}, y2={crater_info["y2"]}')
            self.get_logger().warn(f'  Confidence: {crater_info["confidence"]:.3f}')
            self.get_logger().warn(f'  Size (pixels): {width_pixels}px × {height_pixels}px')
            self.get_logger().warn(f'  Size (meters): {self.crater_width:.2f}m × {self.crater_length:.2f}m')
            self.get_logger().warn('  Initiating right-side bypass maneuver...')
            self.get_logger().warn('  Nav2 continues planning in background')
            self.get_logger().warn('='*60)
            
            # Initiate avoidance maneuver
            self.start_avoidance()
    
    def start_avoidance(self):
        """Initiate crater avoidance sequence."""
        self.avoidance_state = AvoidanceState.MOVE_BACK
        self.maneuver_start_time = time.time()
        
        # Calculate duration to move back based on backward_distance
        self.current_maneuver_duration = self.backward_dist / self.avoid_linear
        
        self.get_logger().info(f'Step 1/5: MOVE_BACK {self.backward_dist:.2f}m')
    
    def avoidance_state_machine(self):
        """Execute crater avoidance maneuver state machine."""
        if self.avoidance_state == AvoidanceState.NORMAL:
            return
        
        # Check if we should abort (switched to manual)
        if not self.is_autonomous:
            self.avoidance_state = AvoidanceState.NORMAL
            return
        
        elapsed = time.time() - self.maneuver_start_time
        vel = Velocity()
        
        if self.avoidance_state == AvoidanceState.MOVE_BACK:
            # Move back 0.2m
            if elapsed < self.current_maneuver_duration:
                vel.vx = 0.0
                vel.vy = -self.avoid_linear  # Negative = backward
                vel.vw = 0.0
                self.vel_pub.publish(vel)
            else:
                self.transition_to_turn_right()
        
        elif self.avoidance_state == AvoidanceState.TURN_RIGHT:
            # Turn right 90 degrees
            if elapsed < self.current_maneuver_duration:
                vel.vx = 0.0
                vel.vy = 0.0
                vel.vw = -self.avoid_angular  # Negative = clockwise/right
                self.vel_pub.publish(vel)
            else:
                self.transition_to_move_right()
        
        elif self.avoidance_state == AvoidanceState.MOVE_RIGHT:
            # Move forward by crater width
            if elapsed < self.current_maneuver_duration:
                vel.vx = 0.0
                vel.vy = self.avoid_linear  # Forward in current orientation
                vel.vw = 0.0
                self.vel_pub.publish(vel)
            else:
                self.transition_to_turn_left()
        
        elif self.avoidance_state == AvoidanceState.TURN_LEFT:
            # Turn left 90 degrees
            if elapsed < self.current_maneuver_duration:
                vel.vx = 0.0
                vel.vy = 0.0
                vel.vw = self.avoid_angular  # Positive = counter-clockwise/left
                self.vel_pub.publish(vel)
            else:
                self.transition_to_move_forward()
        
        elif self.avoidance_state == AvoidanceState.MOVE_FORWARD:
            # Move forward by crater length
            if elapsed < self.current_maneuver_duration:
                vel.vx = 0.0
                vel.vy = self.avoid_linear  # Forward
                vel.vw = 0.0
                self.vel_pub.publish(vel)
            else:
                self.complete_avoidance()
    
    def transition_to_turn_right(self):
        """Transition to turning right."""
        self.avoidance_state = AvoidanceState.TURN_RIGHT
        self.maneuver_start_time = time.time()
        # 90 degrees = ?/2 radians
        self.current_maneuver_duration = (math.pi / 2) / self.avoid_angular
        self.get_logger().info('Step 2/5: TURN_RIGHT 90°')
    
    def transition_to_move_right(self):
        """Transition to moving right (along crater width)."""
        self.avoidance_state = AvoidanceState.MOVE_RIGHT
        self.maneuver_start_time = time.time()
        self.current_maneuver_duration = self.crater_width / self.avoid_linear
        self.get_logger().info(f'Step 3/5: MOVE_RIGHT {self.crater_width:.2f}m')
    
    def transition_to_turn_left(self):
        """Transition to turning left."""
        self.avoidance_state = AvoidanceState.TURN_LEFT
        self.maneuver_start_time = time.time()
        self.current_maneuver_duration = (math.pi / 2) / self.avoid_angular
        self.get_logger().info('Step 4/5: TURN_LEFT 90°')
    
    def transition_to_move_forward(self):
        """Transition to moving forward (along crater length)."""
        self.avoidance_state = AvoidanceState.MOVE_FORWARD
        self.maneuver_start_time = time.time()
        self.current_maneuver_duration = self.crater_length / self.avoid_linear
        self.get_logger().info(f'Step 5/5: MOVE_FORWARD {self.crater_length:.2f}m')
    
    def complete_avoidance(self):
        """Complete avoidance maneuver and resume normal operation."""
        self.avoidance_state = AvoidanceState.NORMAL
        
        self.get_logger().info('='*60)
        self.get_logger().info('CRATER AVOIDANCE COMPLETE')
        self.get_logger().info('  Resuming Nav2 control')
        self.get_logger().info('  Nav2 will continue to goal automatically')
        self.get_logger().info('='*60)
        
        # Publish brief stop command, then Nav2 takes over
        self.publish_stop()
    
    def publish_stop(self):
        """Publish stop command."""
        vel = Velocity()
        vel.vx = 0.0
        vel.vy = 0.0
        vel.vw = 0.0
        self.vel_pub.publish(vel)
    
    def twist_callback(self, msg: Twist):
        """Convert geometry_msgs/Twist to robot_interfaces/Velocity."""
        # Store the last cmd_vel
        self.last_cmd_vel = msg
        
        # Only process cmd_vel when in NORMAL state (not avoiding crater)
        if self.avoidance_state != AvoidanceState.NORMAL:
            # During crater avoidance, ignore cmd_vel from Nav2
            # Nav2 keeps running and planning, but we override its commands
            return
        
        # Normal operation - pass through Nav2's commands
        vel = Velocity()
        
        # Nav2 Twist convention (standard ROS):
        # linear.x > 0 = forward
        # angular.z > 0 = counter-clockwise
        
        vel.vx = 0.0  # Differential drive can't strafe
        vel.vy = self.clamp(msg.linear.x, -self.max_linear, self.max_linear)
        vel.vw = self.clamp(msg.angular.z, -self.max_angular, self.max_angular)
        
        self.vel_pub.publish(vel)
    
    def clamp(self, value: float, min_val: float, max_val: float) -> float:
        """Clamp value between min and max."""
        return max(min_val, min(max_val, value))

def main(args=None):
    rclpy.init(args=args)
    node = VelocityBridge()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()