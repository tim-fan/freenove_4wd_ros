#!/bin/bash

SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )

# assuming this script is contained in colcon_ws/src/freenove_4wd_ros/systemd
WS_SETUP=${SCRIPT_DIR}/../../../install/setup.bash
DRIVE_CONFIG=${SCRIPT_DIR}/../config/drive_config_backward.yaml

source ${SCRIPT_DIR}/../../../install/setup.bash

ros2 launch freenove_4wd_ros gamepad_teleop.launch.py 