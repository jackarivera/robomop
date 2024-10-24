# Wheel Radius = 0.0725m
# Wheel Base = 0.375m
# x-velocity max = 1 m/s
# ang-velocity max = 5 rad/s
# robomop_driver.py
import rclpy
import re
from rclpy.node import Node
from std_msgs.msg import String, Float32, Float32MultiArray
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
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


        # Publisher for IMU data
        self.imu_publisher = self.create_publisher(Imu, 'robomop/imu_data', 10)

        # Publisher for chassis Odometry
        self.chassis_odometry_publisher = self.create_publisher(Odometry, 'robomop/chassis_odometry', 10)

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

        # Define the pattern for parsing
        pattern = r"RESP\|WHEEL_LEFT\|([-\d.]+)\|WHEEL_RIGHT\|([-\d.]+)\|DISTANCE\|([-\d.]+)\|IMU_AX\|([-\d.]+)\|IMU_AY\|([-\d.]+)\|IMU_AZ\|([-\d.]+)"
        match = re.match(pattern, data.strip())

        if match:
            try:
                # Extract sensor data
                left_wheel_speed = float(match.group(1))   # in turns/s
                right_wheel_speed = float(match.group(2))  # in turns/s
                distance = float(match.group(3))
                imu_ax = float(match.group(4))            # in m/s²
                imu_ay = float(match.group(5))            # in m/s²
                imu_az = float(match.group(6))            # in m/s²

                # Publish IMU data
                imu_msg = Imu()
                imu_msg.linear_acceleration.z = imu_az

                # If you have orientation and angular velocity data, populate them here
                # For now, they are left as default (zeros)

                imu_msg.header.stamp = self.get_clock().now().to_msg()
                imu_msg.header.frame_id = "imu_link"  # Update as per your TF configuration

                self.imu_publisher.publish(imu_msg)

                # Compute chassis speeds using differential drive kinematics
                x_vel, angular_vel = self.diff_drive.computeChassisSpeeds(left_wheel_speed, right_wheel_speed)

                self.get_logger().debug(f"Computed chassis speeds - X: {x_vel} m/s, Angular: {angular_vel} rad/s")

                # Create and publish Odometry message
                odom_msg = Odometry()
                odom_msg.header.stamp = self.get_clock().now().to_msg()
                odom_msg.header.frame_id = "odom"  # Update as per your TF configuration
                odom_msg.child_frame_id = "chassis_icr_link"

                # Pose is not computed here; set to zero or inherit from TF if necessary
                odom_msg.pose.pose.position.x = 0.0
                odom_msg.pose.pose.position.y = 0.0
                odom_msg.pose.pose.position.z = 0.0
                odom_msg.pose.pose.orientation.x = 0.0
                odom_msg.pose.pose.orientation.y = 0.0
                odom_msg.pose.pose.orientation.z = 0.0
                odom_msg.pose.pose.orientation.w = 1.0

                # Twist data
                odom_msg.twist.twist.linear.x = x_vel
                odom_msg.twist.twist.linear.y = 0.0
                odom_msg.twist.twist.linear.z = 0.0
                odom_msg.twist.twist.angular.x = 0.0
                odom_msg.twist.twist.angular.y = 0.0
                odom_msg.twist.twist.angular.z = angular_vel

                # Pose and Twist covariance can be set to high uncertainty if not used
                odom_msg.pose.covariance = [1e6]*36
                odom_msg.twist.covariance = [0.05, 0, 0, 0, 0, 0,
                                             0, 0.05, 0, 0, 0, 0,
                                             0, 0, 1e6, 0, 0, 0,
                                             0, 0, 0, 1e6, 0, 0,
                                             0, 0, 0, 0, 1e6, 0,
                                             0, 0, 0, 0, 0, 0.1]

                self.chassis_odometry_publisher.publish(odom_msg)
                

                self.get_logger().info("Published IMU data and chassis Twist.")
            except ValueError as e:
                self.get_logger().error(f"Error parsing sensor data: {e}")
        else:
            self.get_logger().warn(f"Failed to parse serial data: {data}")

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
