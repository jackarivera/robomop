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
        left_wheel_turns = left_wheel_ms / (self.wheel_radius * 2 *math.pi)
        right_wheel_turns = right_wheel_ms / (self.wheel_radius * 2 *math.pi)

        return (left_wheel_turns, right_wheel_turns)

