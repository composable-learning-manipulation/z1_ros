#!/usr/bin/env python3

# Copyright (C) 2015 Fetch Robotics Inc.
# Copyright (C) 2013-2014 Unbounded Robotics Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
#
# Program has a few changes to do compatible it with Unitree z1 robot:
# - Add support for custom output .bag filename using argparse
# - Adapted for python3 
#
# Contributor: Diuzhev Vladislav


from __future__ import print_function

import time

import rospy
import rosbag

from sensor_msgs.msg import JointState
from robot_calibration_msgs.msg import CaptureConfig
import argparse

NAMES = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
POS = [
    [0.105502, 0.835049, -0.606590, -0.016419, -0.001394, 0.002219],
    [-1.076982, 1.585304, -0.790463, -0.681258, 0.581049, 0.007969],
    [-0.881202, 0.591128, -0.353571, -0.357496, 0.580633, 0.007960],
    [-0.750436, 1.893570, -0.628616, -1.706012, 0.570766, 0.007959],
    [-1.983703, 2.189280, -1.207951, -1.515131, 1.433948, 0.636980],
    [-1.118732, 1.183300, -0.318255, -0.927581, 1.043796, 1.120501],
    [-0.171501, 1.430866, -0.447869, -0.973742, -0.630872, 1.589037],
    [0.268906, 0.800218, -0.139478, -1.118612, -0.608820, 1.588308],
    [-0.407331, 1.676453, -0.914781, -0.598618, -0.613179, 0.955685],
    [-0.401067, 1.801751, -0.785238, -1.088249, -0.612359, 2.259370],
    [-0.636407, 1.530003, -1.249587, 0.480968, 0.291891, 2.259035],
    [-0.361940, 0.715145, -0.733279, 0.481599, 0.291890, 2.259033],
    [-1.371131, 1.169245, -0.427580, -0.562804, 0.532994, 2.259053],
    [-0.699803, 2.052715, -1.132642, -1.190277, 0.532387, 1.394949],
    [-0.608022, 0.655184, -0.061975, -1.172941, 0.532356, 1.394952]
]

class CapturePoses:
    last_state_ = None # last joint states

    def __init__(self):
        self.last_state_ = CaptureConfig()

        for name in NAMES:
            self.last_state_.joint_states.name.append(name)

        rospy.init_node('capture_calibration_poses')

        # bag to write data to
        bag = rosbag.Bag('calibration_poses.bag', 'w')

        # put samples in
        count = 0
        while not rospy.is_shutdown():
            for pose in POS:
                time.sleep(2)
                for j in range(len(NAMES)):
                    if len(self.last_state_.joint_states.position) > j:
                        self.last_state_.joint_states.position[j] = pose[j]
                    else:
                        self.last_state_.joint_states.position.append(pose[j])
                # save a pose
                print(self.last_state_)
                bag.write('calibration_joint_states', self.last_state_)
                print("Saving pose %d" % count)
                count += 1
            break
        bag.close()


if __name__ == "__main__":
    # Parse the arguments
    parser = argparse.ArgumentParser(description="Calibrate the robot, update files in /etc/ros")
    parser.add_argument("--output", type=str, help="Output .bag filename and path",
                        default="calibration_poses.bag")

    args = parser.parse_args()

    CapturePoses()