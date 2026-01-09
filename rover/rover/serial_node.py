#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import serial
import struct
from robot_interfaces.msg import (
    BnoReading, EncoderRaw, LimitSwitches, 
    ModeSwitch, Counter, Velocity, ActuatorCmd
)

class SerialBridgeNode(Node):
    def __init__(self):
        super().__init__('serial_bridge_node')
        
        # Declare parameters
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('baud_rate', 115200)
        
        # Get parameters
        port = self.get_parameter('serial_port').value
        baud = self.get_parameter('baud_rate').value
        
        # Initialize serial connection
        try:
            self.serial = serial.Serial(port, baud, timeout=0.1)
            self.get_logger().info(f'Connected to {port} at {baud} baud')
        except Exception as e:
            self.get_logger().error(f'Failed to open serial port: {e}')
            raise
        
        # Publishers for sensor data
        self.bno_pub = self.create_publisher(BNOReading, '/bno', 10)
        self.encoder_pub = self.create_publisher(EncoderRaw, '/encoder_raw', 10)
        self.limit_pub = self.create_publisher(LimitSwitch, '/limit_switch', 10)
        self.mode_pub = self.create_publisher(ModeSwitch, '/mode_switch', 10)
        self.counter_pub = self.create_publisher(CounterSwitch, '/counter_switch', 10)
        
        # Subscribers for command data
        self.velocity_sub = self.create_subscription(
            Velocity, '/velocity', self.velocity_callback, 10)
        self.actuator_sub = self.create_subscription(
            ActuatorCmd, '/actuator_cmd', self.actuator_callback, 10)
        
        # Protocol constants
        self.START_BYTE = 0xAA
        self.MSG_BNO = 0x01
        self.MSG_ENCODER = 0x02
        self.MSG_LIMIT = 0x03
        self.MSG_MODE = 0x04
        self.MSG_COUNTER = 0x05
        self.MSG_VELOCITY = 0x10
        self.MSG_ACTUATOR = 0x11
        
        # Create timer for reading serial data
        self.create_timer(0.001, self.read_serial_data)  # 1ms = 1kHz
        
        self.get_logger().info('Serial bridge node initialized')
    
    def read_serial_data(self):
        """Read and parse incoming serial data"""
        try:
            while self.serial.in_waiting > 0:
                # Look for start byte
                byte = self.serial.read(1)
                if len(byte) == 0 or byte[0] != self.START_BYTE:
                    continue
                
                # Read message type
                msg_type_byte = self.serial.read(1)
                if len(msg_type_byte) == 0:
                    continue
                msg_type = msg_type_byte[0]
                
                # Read data length
                data_len_byte = self.serial.read(1)
                if len(data_len_byte) == 0:
                    continue
                data_len = data_len_byte[0]
                
                # Read data
                data = self.serial.read(data_len)
                if len(data) != data_len:
                    continue
                
                # Read checksum
                checksum_byte = self.serial.read(1)
                if len(checksum_byte) == 0:
                    continue
                received_checksum = checksum_byte[0]
                
                # Verify checksum
                calculated_checksum = msg_type ^ data_len
                for b in data:
                    calculated_checksum ^= b
                
                if calculated_checksum != received_checksum:
                    self.get_logger().warn('Checksum mismatch')
                    continue
                
                # Process message
                self.process_message(msg_type, data)
                
        except Exception as e:
            self.get_logger().error(f'Error reading serial: {e}')
    
    def process_message(self, msg_type, data):
        """Process received message based on type"""
        try:
            if msg_type == self.MSG_BNO and len(data) == 12:
                msg = BNOReading()
                msg.yaw, msg.pitch, msg.roll = struct.unpack('<fff', data)
                self.bno_pub.publish(msg)
                
            elif msg_type == self.MSG_ENCODER and len(data) == 16:
                msg = EncoderRaw()
                msg.fl_ticks, msg.fr_ticks, msg.bl_ticks, msg.br_ticks = \
                    struct.unpack('<iiii', data)
                self.encoder_pub.publish(msg)
                
            elif msg_type == self.MSG_LIMIT and len(data) == 3:
                msg = LimitSwitch()
                msg.ls1 = bool(data[0])
                msg.ls2 = bool(data[1])
                msg.ls3 = bool(data[2])
                self.limit_pub.publish(msg)
                
            elif msg_type == self.MSG_MODE and len(data) == 1:
                msg = ModeSwitch()
                msg.autonomous = bool(data[0])
                self.mode_pub.publish(msg)
                
            elif msg_type == self.MSG_COUNTER and len(data) == 12:
                msg = CounterSwitch()
                msg.a, msg.b, msg.c, msg.d = struct.unpack('<hhff', data)
                self.counter_pub.publish(msg)
                
        except Exception as e:
            self.get_logger().error(f'Error processing message: {e}')
    
    def velocity_callback(self, msg):
        """Send velocity command to ESP32"""
        try:
            data = struct.pack('<fff', msg.vy, msg.vw, msg.vx)
            self.send_message(self.MSG_VELOCITY, data)
        except Exception as e:
            self.get_logger().error(f'Error sending velocity: {e}')
    
    def actuator_callback(self, msg):
        """Send actuator command to ESP32"""
        try:
            data = struct.pack('<BB', msg.lead_screw, msg.tub_angle)
            self.send_message(self.MSG_ACTUATOR, data)
        except Exception as e:
            self.get_logger().error(f'Error sending actuator: {e}')
    
    def send_message(self, msg_type, data):
        """Send message to ESP32 via serial"""
        data_len = len(data)
        checksum = msg_type ^ data_len
        for b in data:
            checksum ^= b
        
        packet = bytes([self.START_BYTE, msg_type, data_len]) + data + bytes([checksum])
        self.serial.write(packet)
    
    def __del__(self):
        """Cleanup on node shutdown"""
        if hasattr(self, 'serial') and self.serial.is_open:
            self.serial.close()

def main(args=None):
    rclpy.init(args=args)
    node = SerialBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()