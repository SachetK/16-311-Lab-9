from motorgo import Plink
import time
import math
import numpy as np
from collections import deque

# === PID GAINS (scaled to prevent overpowering) ===
KP_ARM_1 = 3.0
KI_ARM_1 = 0
KD_ARM_1 = 0.05

KP_ARM_2 = 5.0
KI_ARM_2 = 0.4
KD_ARM_2 = 0.05

# === ROBOT GEOMETRY ===
l1 = 3.75  # Length of arm1
l2 = 2.5   # Length of arm2
GEAR_RATIO = 3  # 1 motor rev = 1/4 joint rev

def angle_diff_rad(target, current):
    """Returns shortest angular difference in radians, in [-π, π)."""
    return (target - current + np.pi) % (2 * np.pi) - np.pi

# === Motor Wrapper ===
class Motor:
    def __init__(self, plink_channel, gear_ratio=1.0):
        self.motor = plink_channel
        self.gear_ratio = gear_ratio
        self.encoder_offset = 0.0

    def reset_encoder(self, wait_for_stable=True, timeout=2.0, tolerance=0.01):
        if wait_for_stable:
            print("Waiting for encoder to stabilize...")
            start = time.time()
            last_pos = self.motor.position
            while time.time() - start < timeout:
                time.sleep(0.05)
                new_pos = self.motor.position
                if new_pos != 0.0 and abs(new_pos - last_pos) < tolerance:
                    break
                last_pos = new_pos
            print(f"Encoder stabilized at {new_pos:.2f}")
        self.encoder_offset = self.motor.position

    @property
    def position(self):
        return (self.motor.position - self.encoder_offset) / self.gear_ratio

    @property
    def velocity(self):
        return self.motor.velocity / self.gear_ratio

    @property
    def power(self):
        return self.motor.power_command
    
    @property
    def voltage_limit(self):
        return self.motor.motor_voltage_limit
    
    @voltage_limit.setter
    def voltage_limit(self, value):
        self.motor.motor_voltage_limit = value 

    @power.setter
    def power(self, value):
        self.motor.power_command = np.clip(value, -1.0, 1.0)

class Robot:
    def __init__(self):
        self.angle1 = 0.0 # Starting at 0°
        self.angle2 = 0.0
        self.x = 6.25
        self.y = 0.0 

    def update_position(self):
        x = l1 * np.cos(self.angle1) + l2 * np.cos(self.angle2)
        y = l1 * np.sin(self.angle1) + l2 * np.sin(self.angle2)
        self.x, self.y = x, y

    def move_arm1(self, target_angle1, dt, arm1):
        tolerance = math.radians(0.05)
        max_time = 2.0
        start = time.time()

        error_window = deque(maxlen=15)
        prev_error = 0.0
        
        self.angle1 = arm1.position
        error = target_angle1 - self.angle1
        
        while abs(error) > tolerance and time.time() - start < max_time:
            self.angle1 = arm1.position
            print(f"Moved to ({math.degrees(self.angle1):.1f}°,\
                    {math.degrees(self.angle2):.1f}°)")
            
            error = target_angle1 - self.angle1
            
            error_window.append(error)
            integral = sum(error_window) * dt
            derivative = (error - prev_error) / dt
            prev_error = error

            pid_output = KP_ARM_1 * error + KI_ARM_1 * integral + KD_ARM_1 * derivative

            arm1_cmd = np.clip(pid_output, -0.4, 0.4)
            arm1.power = arm1_cmd

            print(f"θ1: {math.degrees(self.angle1):.1f}°, Target: {math.degrees(target_angle1):.1f}°")

            time.sleep(dt)

        arm1.power_command = 0

    def move_arm2(self, target_angle2, dt, arm2):
        tolerance = math.radians(0.05)
        max_time = 2.0
        start = time.time()
    
        error_window = deque(maxlen=15)
        prev_error = 0.0
    
        error = target_angle2 - self.angle2
        
        while abs(error) > tolerance and time.time() - start < max_time:
            self.angle2 = -arm2.position
    
            error = angle_diff_rad(target_angle2, self.angle2)
           
            error_window.append(error)
            integral = sum(error_window) * dt
            derivative = (error - prev_error) / dt
            prev_error = error

            pid_output = KP_ARM_2 * error + KI_ARM_2 * integral + KD_ARM_2 * derivative
            
            arm2_cmd = np.clip(pid_output, -0.4, 0.4)
            arm2.power = -arm2_cmd
    
            print(f"θ2: {math.degrees(self.angle2):.1f}°, Target: {math.degrees(target_angle2):.1f}°")
    
            time.sleep(dt)
    
        arm2.power_command = 0

    def go_to_pose(self, target_angle1, target_angle2, dt, arm1, arm2):
        self.move_arm1(target_angle1, dt, arm1)
        self.move_arm2(target_angle2, dt, arm2)
        self.update_position()

def read_angle_sequence(filename):
    with open(filename, 'r') as f:
        return [(math.radians(float(a1)), math.radians(float(a2)))
                for a1, a2 in (line.strip().split() for line in f)]

def main():
    plink = Plink()
    arm1 = Motor(plink.channel3, gear_ratio=GEAR_RATIO)
    arm2 = Motor(plink.channel4, gear_ratio=GEAR_RATIO)

    arm1.voltage_limit = 6.0
    arm2.voltage_limit = 6.0
    plink.connect()

    robot = Robot()
    angle_sequence = read_angle_sequence('angles.txt')

    arm1.reset_encoder()
    arm2.reset_encoder()

    try:
#        while True:
#            print(f"Arm 1 pos: {math.degrees(arm1.position):.1f}, Arm 2 pos: {-math.degrees(arm2.position):.1f}")
#            time.sleep(0.02)

        for a1, a2 in angle_sequence:
            robot.go_to_pose(a1, a2, 0.02, arm1, arm2)
            print(f"Moved to ({math.degrees(arm1.position):.1f}°, {math.degrees(arm2.position):.1f}°) → Position: ({robot.x:.2f}, {robot.y:.2f})")

    except KeyboardInterrupt:
        print("Stopped by user.")
    finally:
        arm1.power = 0
        arm2.power = 0

if __name__ == "__main__":
    main()
