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

import time
import math, csv
import os
from argparse import ArgumentParser
from pathlib import Path
from datetime import datetime

import rclpy
from rclpy.node import Node
#from rclpy.executors import MultiThreadedExecutor
from rclpy.executors import ExternalShutdownException
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
#from std_srvs.srv import SetBool
from std_msgs.msg import String, UInt8, Bool
from nav_msgs.msg import Odometry
from autoware_planning_msgs.msg import Trajectory

from scenario_test_runner_msgs.srv import TriggerLogging


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

class Logger(Node):
    def __init__(self, output_directory):
        super().__init__('v_logging_node', enable_rosout=False)
        self.get_logger().set_level(rclpy.logging.LoggingSeverity.INFO)

        #self.cnt = 0
        self.output_directory = output_directory
        self.vlf = None
        self.plf = None

        # Setting a trigger service.
        self.srv = self.create_service(TriggerLogging, 'trigger_logger', self.trigger_cb)
        self.get_logger().info(f"service server for trigger_logger ready")

        # Setting subscriptions.
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

    def trigger_cb(self, request, response):
        response.success = True
        if request.message:
            if (self.vlf and not self.vlf.closed) or (self.plf and not self.plf.closed):
                response.success = False
                response.message = 'file handles are not closed properly'
            else:
                #self.cnt += 1
                #out_dir = os.path.join(self.output_directory, str(self.cnt))
                out_dir = self.output_directory / request.message
                if not os.path.exists(out_dir):
                    os.mkdir(out_dir)

                # Setting log files.
                fn = os.path.join(out_dir, VL_FILENAME)
                self.get_logger().info(f"preparing: {fn}")
                self.vlf = open(fn, 'w')

                fn = os.path.join(out_dir, PL_FILENAME)
                self.get_logger().info(f"preparing: {fn}")
                self.plf = open(fn, 'w')

                response.message = 'file handles are prepared'
        else:
            # 
            if self.vlf and not self.vlf.closed:
                self.vlf.flush() 
                self.vlf.close() 
            if self.plf and not self.plf.closed:
                self.plf.flush() 
                self.plf.close() 

            response.message = 'file handles are closed'

        return response

    def vehicle_log_cb(self, msg):
        if not self.vlf or self.vlf.closed:
            return 
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        ori = msg.pose.pose.orientation
        yaw = quaternion_to_yaw(ori.x, ori.y, ori.z, ori.w)
        ts = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
        csv.writer(self.vlf).writerow([ts, x, y, yaw])

    def planning_log_cb(self, msg):
        if not self.vlf or self.plf.closed:
            return 
        ts = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
        writer = csv.writer(self.plf)
        for pt in msg.points:
            x = pt.pose.position.x
            y = pt.pose.position.y
            ori = pt.pose.orientation
            yaw = quaternion_to_yaw(ori.x, ori.y, ori.z, ori.w)
            v = pt.longitudinal_velocity_mps
            writer.writerow([ts, x, y, yaw, v])

#

def main(args=None):

    rclpy.init(args=args)

    parser = ArgumentParser()
    parser.add_argument("--output-directory", default=Path("/tmp"), type=Path)
    parser.add_argument("--output-subdir-name", default="test_runner", type=str)
    parser.add_argument("--ros-args", nargs="*")  # XXX DIRTY HACK
    parser.add_argument("-r", nargs="*")  # XXX DIRTY HACK
    args = parser.parse_args()

    # Prepare a log dir.
    #now = datetime.now()
    #datetime_str = now.strftime('%Y-%m-%d-%H-%M-%S')
    #dirname = f"mod-{datetime_str}"
    #log_dir = os.path.join(args.output_directory, dirname)
    #if not os.path.exists(log_dir):
    #    os.mkdir(log_dir)

    output = args.output_directory / args.output_subdir_name
    if output.exists():
        for each in output.iterdir():
            each.resolve().unlink()
    else:
        output.mkdir(parents=True, exist_ok=True)

    logger = Logger(output)

    try:
        rclpy.spin(logger)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        logger.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    """Entrypoint."""
    main()
