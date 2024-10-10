# Wheel Radius = 0.0725m
# Wheel Base = 0.375m
# x-velocity max = 1 m/s
# ang-velocity max = 5 rad/s
# robomop_driver.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32, Float32MultiArray
from geometry_msgs.msg import Twist
from robomop_driver.utilities.serial_handler import SerialHandler
from robomop_driver.utilities.diff_drive_base import DiffDrive

class RobomopDriverNode(Node):
    def cmd_vel_timeout_callback(self):
        """
        Callback function to check if cmd_vel messages have timed out.
        If timed out, send stop command to Arduino.
        """
        current_time = self.get_clock().now()
        time_since_last_cmd = (current_time - self.last_cmd_vel_time).nanoseconds / 1e9

        if time_since_last_cmd > self.cmd_vel_timeout:
            self.get_logger().warn("cmd_vel timeout. Sending stop command to Arduino.")
            stop_command = "CMD|PUMP_OFF|SET_LEFT_MOTOR|0|SET_RIGHT_MOTOR|0|SET_BRUSH_MOTORS|0,0"
            self.serial_handler.send(stop_command)
            
    def __init__(self):
        super().__init__('robomop_driver_node')

        # Declare and get parameters
        self.declare_parameter('serial_port', '/dev/ROBOMOP_ARDUINO')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('wheel_radius', 0.0725)  # in meters
        self.declare_parameter('wheel_base', 0.375)     # distance between wheels in meters
        self.declare_parameter('cmd_vel_timeout', 1.0) # seconds

        serial_port = self.get_parameter('serial_port').get_parameter_value().string_value
        baudrate = self.get_parameter('baudrate').get_parameter_value().integer_value
        wheel_radius = self.get_parameter('wheel_radius').get_parameter_value().double_value
        wheel_base = self.get_parameter('wheel_base').get_parameter_value().double_value
        self.cmd_vel_timeout = self.get_parameter('cmd_vel_timeout').get_parameter_value().double_value

        # Initialize Serial Handler
        self.serial_handler = SerialHandler(serial_port, baudrate)
        self.serial_handler.open()

        # Initialize Differential Drive
        self.diff_drive = DiffDrive(wheel_radius, wheel_base)

        # Subscribe to cmd_vel
        self.cmd_vel_subscriber = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10
        )

        # Publisher for serial data (optional)
        self.serial_in_publisher = self.create_publisher(String, 'serial_in', 10)

        # Set serial read callback
        self.serial_handler.read_callback = self.handle_serial_data

        # Initialize last_cmd_vel_time
        self.last_cmd_vel_time = self.get_clock().now()

        # Create a timer to check cmd_vel timeout
        self.timer = self.create_timer(0.5, self.cmd_vel_timeout_callback)

        self.get_logger().info('Robomop Driver Node has been started.')

    def cmd_vel_callback(self, msg: Twist):
        """
        Callback function for cmd_vel subscription.
        Computes wheel speeds and sends commands to Arduino.
        """
        x_vel = msg.linear.x       # Forward velocity (m/s)
        y_vel = msg.linear.y       # Forward y velocity (Used for turning on brush motors)
        angular_vel = msg.angular.z  # Angular velocity (rad/s)
        pump_indicator = msg.angular.x # 0: PUMP OFF 1: PUMP ON

        # Update last_cmd_vel_time
        self.last_cmd_vel_time = self.get_clock().now()

        # Compute wheel speeds (turns per second)
        left_wheel_turns, right_wheel_turns = self.diff_drive.computeWheelSpeeds(x_vel, angular_vel)

        self.get_logger().debug(f"Computed wheel speeds - Left: {left_wheel_turns} turns/s, Right: {right_wheel_turns} turns/s")

        left_speed_clamped = 72 if left_wheel_turns * 24 > 72 else -72 if left_wheel_turns * 24 < -72 else left_wheel_turns * 24
        right_speed_clamped = 72 if right_wheel_turns * 24 > 72 else -72 if right_wheel_turns * 24 < -72 else right_wheel_turns * 24

        # Prepare Command String
        if (pump_indicator == 0):
            command_str = f"CMD|PUMP_OFF|SET_LEFT_MOTOR|{left_speed_clamped:.4f}|SET_RIGHT_MOTOR|{right_speed_clamped:.4f}|SET_BRUSH_MOTORS|{y_vel: .4f},{y_vel: .4f}"
        elif (pump_indicator == 1):
            command_str = f"CMD|PUMP_ON|SET_LEFT_MOTOR|{left_speed_clamped:.4f}|SET_RIGHT_MOTOR|{right_speed_clamped:.4f}|SET_BRUSH_MOTORS|{y_vel: .4f},{y_vel: .4f}"
        else:
            command_str = f"CMD|PUMP_OFF|SET_LEFT_MOTOR|{left_speed_clamped:.4f}|SET_RIGHT_MOTOR|{right_speed_clamped:.4f}|SET_BRUSH_MOTORS|{y_vel: .4f},{y_vel: .4f}"
        
        # Send command over serial
        self.serial_handler.send(command_str)

        self.get_logger().info(f"Sent command to Arduino: {command_str}")

    def handle_serial_data(self, data: str):
        """
        Callback function for handling incoming serial data.
        Publishes the data to a ROS2 topic.
        """
        msg = String()
        msg.data = data
        self.serial_in_publisher.publish(msg)
        self.get_logger().info(f"Received from serial: {data}")

    def destroy_node(self):
        # Send command to stop all motors
        self.serial_handler.send("CMD|PUMP_OFF|SET_LEFT_MOTOR|0|SET_RIGHT_MOTOR|0|SET_BRUSH_MOTORS|0,0")
        
        # Close the serial connection
        self.serial_handler.close()
        
        # Call the superclass method to properly destroy the node
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = RobomopDriverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
