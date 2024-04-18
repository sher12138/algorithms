#!/usr/bin/env python3

# ****************************************************************************
# Copyright 2019 The neodrive Authors. All Rights Reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
#  You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
# ****************************************************************************
# -*- coding: utf-8 -*-
"""Module for example of talker."""

import time
import sys

sys.path.append("/home/caros/cyberrt/python")
sys.path.append("/home/caros/cyberrt/python/cyber_py3")
print("sys.path:", sys.path)

from cyber_py3 import cyber
from planning_interface_pb2 import PlanningInterface

import signal

def exit(signum, frame):
    print("exit program")
    exit()

def test_talker_class():
    """
    Test talker.
    """
    msg = PlanningInterface()
    msg.header.timestamp_sec = time.time()
    msg.state = 3
    test_node = cyber.Node("node_name1")
    g_count = 1

    writer = test_node.create_writer("/openapi/planning_interface", PlanningInterface, 6)
    while not cyber.is_shutdown():
        temp_str = input('input any key to send finsh:')
        if temp_str == 'w':
            msg.state = 2
        elif temp_str == 'f':
            msg.state = 3
        elif temp_str == 'c':
            msg.state = 1
        else:
            print("error input [w|f|c]:", temp_str)
            continue
        print(msg)
        writer.write(msg)


if __name__ == '__main__':
    cyber.init("talker_sample")
    signal.signal(signal.SIGINT, exit)
    signal.signal(signal.SIGTERM, exit)
    test_talker_class()
    cyber.shutdown()
