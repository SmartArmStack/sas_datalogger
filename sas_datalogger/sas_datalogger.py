#!/usr/bin/python3

# Copyright (c) 2012-2026 Murilo Marques Marinho
#
#    This file is part of sas_datalogger.
#
#    sas_datalogger is free software: you can redistribute it and/or modify
#    it under the terms of the GNU Lesser General Public License as published by
#    the Free Software Foundation, either version 3 of the License, or
#    (at your option) any later version.
#
#    sas_datalogger is distributed in the hope that it will be useful,
#    but WITHOUT ANY WARRANTY; without even the implied warranty of
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#    GNU Lesser General Public License for more details.
#
#    You should have received a copy of the GNU Lesser General Public License
#    along with sas_datalogger.  If not, see <https://www.gnu.org/licenses/>.
#
# ################################################################
#
#   Author: Murilo M. Marinho, email: murilomarinho@ieee.org
#
# ################################################################

"""sas_datalogger package

ROS2 data logging.

Public classes:
- SASDatalogger: an rclpy.Node that subscribes to the
  ``/sas_datalogger/log`` topic (message type ``sas_msgs::msg::LogDatum``)
  and accumulates values in memory.
"""

import datetime

import numpy
import scipy.io as sio

import rclpy
from rclpy.node import Node

from sas_msgs.msg import LogDatum


class SASDatalogger(Node):
    """ROS2 data logger node.

    Subscribes to the ``/sas_datalogger/log`` topic and stores received values
    in an internal dictionary keyed by the message ``name`` field. The stored
    data can be persisted to disk with :meth:`save` or automatically when the
    instance exits a ``with`` block.
    """

    def __init__(self, node_name: str):
        """Create and initialize the SAS data logger node.

        Args:
            node_name: Name to use when creating the underlying rclpy.Node.
        """
        super().__init__(node_name=node_name)

        self.data = {}
        self.subscription_ = self.create_subscription(
            LogDatum,
            "/sas_datalogger/log",
            self.log_callback,
            100) #  If you're storing more than 100 values at each loop, this might need adjustment

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        filename = 'sas_log_{date:%Y_%m_%d_%H_%M_%S}.mat'.format(date=datetime.datetime.now())
        print("Saving log to filename = '{}'.".format(filename))
        sio.savemat(filename, self.data)

    def save(self, filename: str):
        """Persist collected data to a MATLAB ``.mat`` file.

        Args:
            filename: Path to the output ``.mat`` file.
        """
        sio.savemat(filename, self.data)
        self.data = {}

    def log_callback(self, msg: LogDatum):
        """Callback invoked for incoming :class:`sas_msgs.msg.LogDatum` messages.
        """

        # Initialize list for a given variable
        if msg.name in self.data:
            pass
        else:
            self.data[msg.name] = []

        # Append value to dictionary
        if len(msg.value) > 0:
            if len(msg.layout) == 2:
                self.data[msg.name].append(numpy.asarray(msg.value).reshape(msg.layout))
            else:
                self.data[msg.name].append(msg.value)
        else:
            self.data[msg.name].append(msg.strvalue)

        return
