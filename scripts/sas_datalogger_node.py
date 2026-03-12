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

import datetime

import numpy
import scipy.io as sio

import rclpy
from rclpy.node import Node

from sas_msgs.msg import LogDatum


def main(args=None):

    rclpy.init(args=args)
    with SASDatalogger(node_name="sas_datalogger_node") as sas_datalogger:
        print("sas_datalogger_node ready. End it with CTRL+C.")
        try:
            rclpy.spin(sas_datalogger)
        except KeyboardInterrupt:
            print("sas_datalogger_node ended by user with CTRL+C.")
        except Exception as e:
            print(e)


class SASDatalogger(Node):

    def __init__(self, node_name: str):
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
        sio.savemat(filename, self.data)
        self.data = {}

    def log_callback(self, msg: LogDatum):

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


if __name__ == '__main__':
    main()
