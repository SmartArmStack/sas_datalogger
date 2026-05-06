#!/bin/python3

# Copyright (c) 2012-2023 Murilo Marques Marinho
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

# Author: Murilo M. Marinho, email: murilomarinho@ieee.org
import numpy as np

import rclpy
from rclpy.node import Node

from sas_datalogger import DataloggerClient
from sas_core import Clock
from sas_common import rclcpp_init, rclcpp_Node, rclcpp_spin_some, rclcpp_shutdown


def main(args=None):
    """
    An example showing some of the functionalities of the DataloggerClient.

    :param args: Not used directly by the user, but used by ROS2 to configure
    certain aspects of the Node.
    """
    try:
        rclpy.init(args=args)
        rclcpp_init()  # Init rclcpp to use the sas python bindings. They do not use rclpy.
        # However, you can also have rclpy nodes active as long as you manage their spin
        # correctly.
        rclcpp_node = rclcpp_Node("sas_datalogger_client_example_py_rclcpp")
        rclpy_node = Node("sas_datalogger_client_example_py_rclpy")

        rclpy_node.declare_parameter('execution_times', 5)
        execution_times = rclpy_node.get_parameter('execution_times').get_parameter_value().integer_value

        datalogger_client = DataloggerClient(rclcpp_node)

        clock = Clock(0.001)
        clock.init()
        while not datalogger_client.is_enabled():
            rclcpp_spin_some(rclcpp_node)
            clock.update_and_sleep()

        for i in range(0, execution_times):
            A = np.array([[1, 2, 3],
                          [4, 5, 6],
                          [7, 8, 9],
                          [10, 11, 12]])
            datalogger_client.log("A", A)

            v = np.array([1, 5, 10, 15, 20])
            datalogger_client.log("v", v)

            datalogger_client.log("another_value_double", -i)
            datalogger_client.log("value_double", i)

            datalogger_client.log("value_string", "Hello world!")

            rclcpp_spin_some(rclcpp_node)
            clock.update_and_sleep()

    except KeyboardInterrupt:
        print("Interrupted by user")


if __name__ == '__main__':
    main()
