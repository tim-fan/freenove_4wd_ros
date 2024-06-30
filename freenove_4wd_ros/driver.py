from freenove_4wd_ros.freenove4wd import DriveSystem  

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from geometry_msgs.msg import Twist

import sys


class DriverNode(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10)
        self.subscription  # prevent unused variable warning

        # get drive system parameters 
        self.declare_parameter("wheelbase", rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter("max_speed", rclpy.Parameter.Type.DOUBLE)
        wheelbase = self.get_parameter("wheelbase").value
        max_speed = self.get_parameter("max_speed").value

        motor_channel_names = [
            "rear_right_fwd_channel",
            "rear_right_bkwd_channel",
            "front_right_fwd_channel",
            "front_right_bkwd_channel",
            "front_left_fwd_channel",
            "front_left_bkwd_channel",
            "rear_left_fwd_channel",
            "rear_left_bkwd_channel",
        ]

        motor_channels = dict()
        for channel_name in motor_channel_names:
            self.declare_parameter(channel_name, rclpy.Parameter.Type.INTEGER)
            channel_val = self.get_parameter(channel_name).value
            motor_channels[channel_name] = channel_val

        # construct drive system, set motor speeds to zero
        self.drive = DriveSystem(motor_channels, wheelbase, max_speed)
        self.drive.set_speed(0,0)

    def cmd_vel_callback(self, msg:Twist):
        self.drive.set_speed(msg.linear.x, msg.angular.z)


def main(args=None):
    rclpy.init(args=args)
    try:
        node = DriverNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        sys.exit(1)


if __name__ == '__main__':
    main()
