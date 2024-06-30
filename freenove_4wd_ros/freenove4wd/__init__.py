from .PCA9685 import PCA9685
import time
from typing import Tuple
from math import pi, copysign

MAX_DUTY = 4095 # copied from freenove code. Not sure what happens if exceeded
DEG2RAD = pi/180

TEST_MOTOR_CHANNELS ={
    "rear_right_fwd_channel":  1, 
    "rear_right_bkwd_channel": 0,
    "front_right_fwd_channel":  2, 
    "front_right_bkwd_channel": 3,
    "front_left_fwd_channel":  5, 
    "front_left_bkwd_channel": 4,
    "rear_left_fwd_channel":  7, 
    "rear_left_bkwd_channel": 6,
}
TEST_WHEELBASE = 0.2
TEST_MAX_SPEED = 0.4

class Motor:
    def __init__(self, fwd_channel:int, bkwd_channel:int, pwm:PCA9685, max_speed:float=0.4, name="motor"):
        """
        max_speed: how fast the motor travels (m/s) when driven at max duty cycle
        """
        assert fwd_channel != bkwd_channel, "forward and backward channels should not be the same"

        self.fwd_channel = fwd_channel
        self.bkwd_channel = bkwd_channel
        self.max_speed = max_speed

        self._pwm = pwm
        self._speed_scale = MAX_DUTY / max_speed  # scale factor to convert m/s to duty
        self._name = name # for debugging
    
    def set_speed(self, speed:float):
        """
        Take speed in m/s and applies appropriate pwm to motor
        """

        duty = abs(speed * self._speed_scale)
        duty = min(duty, MAX_DUTY) # ensure MAX_DUTY not exceeded
        duty = int(duty) # can only set int-valued duty

        # uncomment for debug
        print(f"{self._name}:\t{speed=},\t{duty=}")

        if speed > 0:
            fwd_duty = duty
            bkwd_duty = 0
        elif speed < 0:
            fwd_duty = 0
            bkwd_duty = duty
        else:
            # zero speed, set both to max duty
            # (motor brake? the example did this
            #  so I'm copying)
            fwd_duty = MAX_DUTY
            bkwd_duty = MAX_DUTY
        
        self._pwm.setMotorPwm(self.fwd_channel, fwd_duty)
        self._pwm.setMotorPwm(self.bkwd_channel, bkwd_duty)

    def __del__(self):
        # set motors to "off" on cleanup
        self.set_speed(0)

class Motors:
    """
    A collection of four motors (front/rear left/right)
    Performs some configuration checks on init
    """
    def __init__(self, front_left:Motor, front_right:Motor, rear_left:Motor, rear_right:Motor):
        self.front_left = front_left
        self.front_right = front_right
        self.rear_left = rear_left
        self.rear_right = rear_right

        # for convenience, record motor speed limit
        # assuming all motors have same limit
        self.max_speed = self.front_left.max_speed
        # check assumption
        for motor in [front_left, front_right, rear_left, rear_right]:
            assert motor.max_speed == self.max_speed, "failed assumption check (all motors should have same speed limit)"

        # basic configuration check - all motors should be using different channels
        channel_users = dict() # channel -> motor name
        motor_names = [
            "front_left",
            "front_right",
            "rear_left",
            "rear_right",
        ]
        for motor_name in motor_names:
            motor:Motor = getattr(self, motor_name)
            for channel in [motor.fwd_channel, motor.bkwd_channel]:
                if channel in channel_users:
                    raise RuntimeError(
                        f"Invalid configuration: motors '{motor_name}' and '{channel_users[channel]}' "
                        f"both use channel {channel}.")
                else:
                    channel_users[channel] = motor_name



class DriveSystem:
    def __init__(self, motor_channels:dict, wheelbase:float=1.0, max_speed:float=0.3):
        """
        Given appropriate drive configuration, construct the drive system.
        Example motor_channels:
            motor_channels = {
                "rear_right_fwd_channel":  1, 
                "rear_right_bkwd_channel": 0,
                "front_right_fwd_channel":  2, 
                "front_right_bkwd_channel": 3,
                "front_left_fwd_channel":  5, 
                "front_left_bkwd_channel": 4,
                "rear_left_fwd_channel":  7, 
                "rear_left_bkwd_channel": 6,
            }
        wheelbase: distance between left and right wheels in metres
        max_speed: how fast the robot travels (m/s) when driven at max duty cycle
        """

        pwm = PCA9685(0x40, debug=True)
        pwm.setPWMFreq(50)

        self.motors = Motors(
            front_left=Motor(
                motor_channels["front_left_fwd_channel"],
                motor_channels["front_left_bkwd_channel"],
                pwm,
                max_speed,
                name="front_left",
            ),
            front_right=Motor(
                motor_channels["front_right_fwd_channel"],
                motor_channels["front_right_bkwd_channel"],
                pwm,
                max_speed,
                name="front_right",
            ),
            rear_left=Motor(
                motor_channels["rear_left_fwd_channel"],
                motor_channels["rear_left_bkwd_channel"],
                pwm,
                max_speed,
                name="rear_left",
            ),
            rear_right=Motor(
                motor_channels["rear_right_fwd_channel"],
                motor_channels["rear_right_bkwd_channel"],
                pwm,
                max_speed,
                name="rear_right",
            ),
        )
        self.wheelbase = wheelbase


    
    def set_speed(self, linear:float, angular:float):
        """
        Apply motor velocities such that system moves 
        with forward velocity "linear" (m/s) and
        angular velocity "angular" (rad/s).
        Forward velocity will be limited if required to
        prevent exceeding motor speed limits.
        """

        # if angular vel less than 5 degrees per sec, treat as 
        # straight line driving
        # (special case required to avoid divide-by-zero in curvature calculations)
        driving_straight = abs(angular) < 5*DEG2RAD
        if driving_straight:
            # left and right wheel velocities are the same
            v_l = linear
            v_r = linear
        else:

            # kinematic model:
            # Just treating as two wheel diff drive, and assigning the output to both
            # wheels on the corresponding side.
            # Calculate radius of curvature for centre wheel (v/w).
            # Use wheelbase to calculate radius of curvature for each
            # wheel.
            # Use wheel radius to calculate wheel velocity 

            r_c = linear / angular       # radius of curvature - robot centre
            r_l = r_c - self.wheelbase/2 # radius of curvature - left wheel
            r_r = r_c + self.wheelbase/2 # radius of curvature - right wheel
            v_l = r_l * angular          # left wheel velocity
            v_r = r_r * angular          # right wheel velocity

        # scale velocities down if needed to avoid exceeding vel limits
        v_l, v_r = self._scale_velocities(v_l, v_r)

        # apply velocities to motors
        self.motors.front_left.set_speed(v_l)
        self.motors.rear_left.set_speed(v_l)
        self.motors.front_right.set_speed(v_r)
        self.motors.rear_right.set_speed(v_r)

    def _scale_velocities(self, v_left_wheel:float, v_right_wheel:float) -> Tuple[float, float]:
        """
        Given wheel velocities, scale down as necessary such that:
            * neither wheel absolute vel exceeds motor speed limits
            * radius of curvature is unchanged
        """
        if v_left_wheel == 0 and v_right_wheel == 0:
            # avoid division by zero 
            scale_factor = 1.0        
        else:
            scale_factor = self.motors.max_speed / max(abs(v_left_wheel), abs(v_right_wheel))
        
        if scale_factor > 1:
            # only scale down (reduce speed), not up
            scale_factor = 1.0
        
        return (v_left_wheel * scale_factor, v_right_wheel * scale_factor)
    


def test_motors():
    
    drive = DriveSystem(TEST_MOTOR_CHANNELS, TEST_WHEELBASE, TEST_MAX_SPEED)

    print("## Individual motor test")
    speed = TEST_MAX_SPEED

    for motor_name in ["front_left", "front_right", "rear_left", "rear_right",]:
        print(motor_name)
        motor = getattr(drive.motors, motor_name)
        motor.set_speed(speed)
        time.sleep(1)
        motor.set_speed(-speed)
        time.sleep(1)
        motor.set_speed(0)
        time.sleep(1)

    print("")
    print("## Drive system test")       

    speed_profile = [
        (f"forward, {speed} m/s",             speed,   0.0),
        ("stop",                              speed,   0.0),
        (f"backward, {speed} m/s",            speed,   0.0),
        ("stop",                              speed,   0.0),
        ("left turn on spot, 90 deg/s",       speed,  90.0*DEG2RAD),
        ("stop",                              speed,   0.0),
        ("right turn on spot, 90 deg/s",      speed, -90.0*DEG2RAD),
        ("stop",                              speed,   0.0),
        ("left turn forward,  90 deg/s",    speed/2,  90.0*DEG2RAD),
        ("stop",                                0.0,   0.0),
        ("right turn backward,  90 deg/s", -speed/2,  -90.0*DEG2RAD),
        ("stop",                                0.0,   0.0),
    ]
    for description, linear, angular in speed_profile:
        print(description)
        drive.set_speed(linear, angular)
        time.sleep(1)
        print()

def test_full_speed_straight():
    """
    Drive forward at full speed.
    Can be used to determine max speed, to determine correct settings for drive system
    """

    max_speed = 1e6 # try go fast, observe how fast it goes

    drive = DriveSystem(TEST_MOTOR_CHANNELS, TEST_WHEELBASE, max_speed)

    print("drive forward indefinitely (ctrl-c to terminate)")
    time.sleep(1)
    drive.set_speed(max_speed, 0)

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("Stopping robot")
        drive.set_speed(0,0)


    