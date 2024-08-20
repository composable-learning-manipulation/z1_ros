#!/usr/bin/env python

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
# Contributor: Antipov Vladislav


from __future__ import print_function

import string

import rospy
import rosbag

from sensor_msgs.msg import JointState
from robot_calibration_msgs.msg import CaptureConfig
import argparse


class CapturePoses:
    last_state_ = None # last joint states

    def __init__(self):
        self.last_state_ = CaptureConfig()

        rospy.init_node('capture_calibration_poses')
        rospy.Subscriber("joint_states", JointState, self.stateCb)

        # bag to write data to
        bag = rosbag.Bag('calibration_poses.bag', 'w')

        # put samples in
        count = 0
        while not rospy.is_shutdown():
            print('Move arm/head to a new sample position:  (type "exit" to quit and save data)')
            resp = input('press <enter>')
            print(resp)
            if resp.upper() == 'EXIT':
                break
            else:
                if self.last_state_ == None:
                    print('Cannot save state')
                    continue
                # save a pose
                print(self.last_state_)
                bag.write('calibration_joint_states', self.last_state_)
                print("Saving pose %d" % count)
                count += 1
        bag.close()

    def stateCb(self, msg):
        """ Callback for joint_states messages """
        for joint, position in zip(msg.name, msg.position):
            try:
                idx = self.last_state_.joint_states.name.index(joint)
                self.last_state_.joint_states.position[idx] = position
            except ValueError:
                self.last_state_.joint_states.name.append(joint)
                self.last_state_.joint_states.position.append(position)

if __name__ == "__main__":
    # Parse the arguments
    parser = argparse.ArgumentParser(description="Calibrate the robot, update files in /etc/ros")
    parser.add_argument("--output", type=str, help="Output .bag filename and path",
                        default="calibration_poses.bag")

    args = parser.parse_args()

    CapturePoses()