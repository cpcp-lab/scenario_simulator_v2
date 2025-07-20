#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Copyright 2025 D. Ishii. All rights reserved.
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

import rclpy
import time
import math, csv
import os
from datetime import datetime
from argparse import ArgumentParser
from rclpy.node import Node
from pathlib import Path
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String, UInt8, Bool
from nav_msgs.msg import Odometry
from autoware_planning_msgs.msg import Trajectory
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

VL_FILENAME = "vehicle_position_log.csv"
PL_FILENAME = "planning_log.csv"

def quaternion_to_yaw(x, y, z, w): 
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp) * 180 / math.pi

log_qos = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    #reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    #depth=100
    depth=1
)

class VLoggingNode(Node):
    def __init__(self, output_directory):
        super().__init__('v_logging_node', enable_rosout=False)
        self.get_logger().set_level(rclpy.logging.LoggingSeverity.INFO)

        # Setting log files.
        self.get_logger().info(f"preparing: {os.path.join(output_directory, VL_FILENAME)}")
        self.vlf = open(os.path.join(output_directory, VL_FILENAME), 'w')

        self.get_logger().info(f"preparing: {os.path.join(output_directory, PL_FILENAME)}")
        self.plf = open(os.path.join(output_directory, PL_FILENAME), 'w')

        self.sub_vehicle_log = self.create_subscription(
            Odometry, '/localization/kinematic_state', 
            self.vehicle_log_cb, log_qos)

        self.sub_planning_log = self.create_subscription(
            Trajectory, '/planning/scenario_planning/trajectory', 
            self.planning_log_cb, log_qos)

    def __del__(self):
        self.get_logger().info('dying...')
        self.vlf.close()
        self.plf.close()
        self.get_logger().info('done')

    def vehicle_log_cb(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        ori = msg.pose.pose.orientation
        yaw = quaternion_to_yaw(ori.x, ori.y, ori.z, ori.w)
        ts = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
        csv.writer(self.vlf).writerow([ts, x, y, yaw])

    def planning_log_cb(self, msg):
        ts = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
        writer = csv.writer(self.plf)
        for pt in msg.points:
            x = pt.pose.position.x
            y = pt.pose.position.y
            ori = pt.pose.orientation
            yaw = quaternion_to_yaw(ori.x, ori.y, ori.z, ori.w)
            v = pt.longitudinal_velocity_mps
            writer.writerow([ts, x, y, yaw, v])

#class PLoggingNode(Node):
#    def __init__(self, output_directory):
#        super().__init__('p_logging_node', enable_rosout=False)
#        self.get_logger().set_level(rclpy.logging.LoggingSeverity.INFO)
#
#        # Setting log files.
#        self.get_logger().info(f"preparing: {os.path.join(output_directory, PL_FILENAME)}")
#        self.plf = open(os.path.join(output_directory, PL_FILENAME), 'w')
#
#        self.sub_planning_log = self.create_subscription(
#            Trajectory, '/planning/scenario_planning/trajectory', 
#            self.planning_log_cb, log_qos)
#
#    def __del__(self):
#        self.get_logger().info('dying...')
#        self.plf.close()
#        self.get_logger().info('done')
#
#    def planning_log_cb(self, msg):
#        ts = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
#        writer = csv.writer(self.plf)
#        for pt in msg.points:
#            x = pt.pose.position.x
#            y = pt.pose.position.y
#            ori = pt.pose.orientation
#            yaw = quaternion_to_yaw(ori.x, ori.y, ori.z, ori.w)
#            v = pt.longitudinal_velocity_mps
#            writer.writerow([ts, x, y, yaw, v])

#

def main(args=None):

    rclpy.init(args=args)

    parser = ArgumentParser()
    parser.add_argument("--output-directory", default=Path("/tmp"), type=Path)
    parser.add_argument("--ros-args", nargs="*")  # XXX DIRTY HACK
    parser.add_argument("-r", nargs="*")  # XXX DIRTY HACK
    args = parser.parse_args()

    # Prepare a log dir.
    now = datetime.now()
    datetime_str = now.strftime('%Y-%m-%d-%H-%M-%S')
    dirname = f"mod-{datetime_str}"
    log_dir = os.path.join(args.output_directory, dirname)
    if not os.path.exists(log_dir):
        os.mkdir(log_dir)

    v_log_node = VLoggingNode(log_dir)
    #p_log_node = PLoggingNode(log_dir)

    #executor = MultiThreadedExecutor()
    #executor.add_node(v_log_node)
    #executor.add_node(p_log_node)
    try:
        rclpy.spin(v_log_node)
    finally:
        v_log_node.destroy_node()
        #p_log_node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    """Entrypoint."""
    main()
