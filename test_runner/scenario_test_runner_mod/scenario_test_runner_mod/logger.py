#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Copyright 2025-2026 D. Ishii. All rights reserved.
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

import math, csv
import os
from argparse import ArgumentParser
from pathlib import Path
from datetime import datetime

import rclpy
from rclpy.time import Time
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry
from autoware_planning_msgs.msg import Trajectory
from autoware_control_msgs.msg import Control
from autoware_internal_debug_msgs.msg import Float32MultiArrayStamped

from scenario_test_runner_msgs.srv import TriggerLogging


# (msg attribute, filename, log function name, immediate, msg type, topic)
CHANNELS = [
    ( 'v_msg', 'vehicle_position_log.csv', 'log_v_msg', False,
      Odometry, '/localization/kinematic_state'),
    ( 'p_msg', 'planning_log.csv',         'log_p_msg', False,
      Trajectory, 
      #'/planning/scenario_planning/trajectory'
      '/planning/trajectory'
      ),
    ( 'c_msg', 'control_log.csv',          'log_c_msg', True,
      Control, '/control/command/control_cmd'),
    ( 'm_msg', 'marker_log.csv',           'log_m_msg', True,
      Float32MultiArrayStamped, '/control/trajectory_follower/controller_node_exe/debug/wp_outputs'),
]

def quaternion_to_yaw(x, y, z, w):
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp) * 180 / math.pi

log_qos = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1
)

class Logger(Node):
    def __init__(self, output_directory):
        super().__init__('v_logging_node', enable_rosout=False)
        self.get_logger().set_level(rclpy.logging.LoggingSeverity.INFO)

        self.output_directory = output_directory
        self.files = {}

        for msg_attr, *_ in CHANNELS:
            setattr(self, msg_attr, None)

        self.plc_count = 0

        self.srv = self.create_service(TriggerLogging, 'trigger_logger', self.trigger_cb)
        self.get_logger().info(f"service server for trigger_logger ready")

        self._setup_subscriptions()

        self.timer = self.create_timer(0.1, self.timer_cb)

    def __del__(self):
        self.get_logger().info('dying...')
        self._close_files()
        self.get_logger().info('done')

    #

    def _close_files(self):
        for f in self.files.values():
            if not f.closed:
                f.flush()
                f.close()
        self.files.clear()

    def _setup_subscriptions(self):
        self.subs = []
        for msg_attr, _, log_fn_name, immediate, msg_type, topic in CHANNELS:
            def callback(msg, msg_attr=msg_attr, log_fn_name=log_fn_name, immediate=immediate):
                f = self.files.get(msg_attr)
                if not f or f.closed:
                    return
                setattr(self, msg_attr, msg)
                if immediate:
                    getattr(self, log_fn_name)(msg)
                    setattr(self, msg_attr, None)
            self.subs.append(
                self.create_subscription(msg_type, topic, callback, log_qos))

    def trigger_cb(self, request, response):
        response.success = True
        if request.message:
            if any(
                (f := self.files.get(attr)) and not f.closed
                for attr, *_ in CHANNELS
            ):
                response.success = False
                response.message = 'file handles are not closed properly'
            else:
                out_dir = self.output_directory / request.message
                if not os.path.exists(out_dir):
                    os.mkdir(out_dir)

                for msg_attr, filename, *_ in CHANNELS:
                    fn = os.path.join(out_dir, filename)
                    self.get_logger().info(f"preparing: {fn}")
                    self.files[msg_attr] = open(fn, 'w')

                response.message = 'file handles are prepared'

            # TO_CHECK
            self.plc_count = 0
        else:
            self._close_files()
            response.message = 'file handles are closed'

        return response

    #

    def log_v_msg(self, msg):
        t = Time.from_msg(msg.header.stamp)
        dt = datetime.fromtimestamp(t.nanoseconds * 1e-9)
        ts = dt.strftime('%Y-%m-%d %H:%M:%S')
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        ori = msg.pose.pose.orientation
        yaw = quaternion_to_yaw(ori.x, ori.y, ori.z, ori.w)
        csv.writer(self.files['v_msg']).writerow([ts, x, y, yaw])

    def log_p_msg(self, msg):
        self.plc_count += 1
        t = Time.from_msg(msg.header.stamp)
        dt = datetime.fromtimestamp(t.nanoseconds * 1e-9)
        ts = dt.strftime('%Y-%m-%d %H:%M:%S.%f')
        writer = csv.writer(self.files['p_msg'])
        for pt in msg.points:
            x = pt.pose.position.x
            y = pt.pose.position.y
            ori = pt.pose.orientation
            yaw = quaternion_to_yaw(ori.x, ori.y, ori.z, ori.w)
            v = pt.longitudinal_velocity_mps
            writer.writerow([self.plc_count, ts, x, y, yaw, v])

    def log_c_msg(self, msg):
        lat = msg.lateral
        t = Time.from_msg(lat.stamp)
        dt = datetime.fromtimestamp(t.nanoseconds * 1e-9)
        ts = dt.strftime('%Y-%m-%d %H:%M:%S.%f')
        sta = lat.steering_tire_angle
        lon = msg.longitudinal
        vel = lon.velocity
        acc = lon.acceleration
        csv.writer(self.files['c_msg']).writerow([ts, sta, vel, acc])

    def log_m_msg(self, msg):
        t = Time.from_msg(msg.stamp)
        dt = datetime.fromtimestamp(t.nanoseconds * 1e-9)
        ts = dt.strftime('%Y-%m-%d %H:%M:%S.%f')
        posX = msg.data[0]
        posY = msg.data[1]
        posYaw = msg.data[2]
        wpX = msg.data[3]
        wpY = msg.data[4]
        wpV = msg.data[5]
        csv.writer(self.files['m_msg']).writerow([ts, posX, posY, posYaw, wpX, wpY, wpV])

    #

    def timer_cb(self):
        for msg_attr, _, log_fn_name, immediate, *_ in CHANNELS:
            if immediate:
                continue
            msg = getattr(self, msg_attr)
            if msg is not None:
                getattr(self, log_fn_name)(msg)
                setattr(self, msg_attr, None)

#

def main(args=None):

    rclpy.init(args=args)

    parser = ArgumentParser()
    parser.add_argument("--output-directory", default=Path("/tmp"), type=Path)
    parser.add_argument("--output-subdir-name", default="test_runner", type=str)
    parser.add_argument("--ros-args", nargs="*")  # XXX DIRTY HACK
    parser.add_argument("-r", nargs="*")  # XXX DIRTY HACK
    args = parser.parse_args()

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
