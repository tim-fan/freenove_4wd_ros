# freenove_4wd_ros

ROS driver for controlling [Freenove 4WD Smart Car Kit for Raspberry Pi](https://github.com/Freenove/Freenove_4WD_Smart_Car_Kit_for_Raspberry_Pi/tree/master?tab=readme-ov-file#freenove-4wd-smart-car-kit-for-raspberry-pi) via [ROS](https://www.ros.org/).

<img src='doc/car.png' width='30%'/>

In this first iteration, just implementing motor drive control via `/cmd_vel` topic. Future work could look at supporting other sensors, odometry etc, but I'm not planning to look at that.

Tested on ROS2 Jazzy, Ubuntu 24.04, Raspberry Pi 4B.

Gamepad control tested with Logitech gamepad F310.

