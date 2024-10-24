import math

class DiffDrive:
    def __init__(self, wheel_radius, wheel_base):
        self.wheel_radius = wheel_radius
        self.wheel_base = wheel_base

    # Takes: Target x velocity, angular velocity relative to robot frame
    # Outputs: Left and Right wheel speeds in turns/s
    def computeWheelSpeeds(self, x_vel, angular_vel):
        # Compute wheel speeds in (m/s)
        left_wheel_ms = x_vel - (0.5 * angular_vel * self.wheel_base)
        right_wheel_ms = x_vel + (0.5 * angular_vel * self.wheel_base)

        # Convert (m/s) to (turns / s)
        left_wheel_turns = left_wheel_ms / (self.wheel_radius * 2 * math.pi)
        right_wheel_turns = right_wheel_ms / (self.wheel_radius * 2 * math.pi)

        return (left_wheel_turns, right_wheel_turns)

    # Takes: Target left and right wheel speeds in turns/s and outputs chassis speeds
    # Outputs: Chassis Speeds in m/s
    def computeChassisSpeeds(self, left_wheel_speed, right_wheel_speed):
        # Compute rad/s for each wheel
        left_rps = (-left_wheel_speed / 24) * 2 * math.pi
        right_rps = (right_wheel_speed / 24) * 2 * math.pi

        # Compute tangential velocity in m/s
        v_r = self.wheel_radius * right_rps
        v_l = self.wheel_radius * left_rps

        # Compute instantaneous body velocities
        x_vel = (v_r + v_l) / 2
        angular_vel = (v_r - v_l) / self.wheel_base
        
        return (x_vel, angular_vel)

