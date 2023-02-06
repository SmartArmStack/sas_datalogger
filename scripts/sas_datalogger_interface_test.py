#!/usr/bin/python3

# Copyright (c) 2012-2021 Murilo Marques Marinho
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
# ################################################################*/

import rospy
import time
import numpy as np
from sas_datalogger import DataloggerInterface

rospy.init_node('sas_datalogger_interface_test', disable_signals=True)

try:

    datalogger = DataloggerInterface(20)
    while not rospy.is_shutdown():
        datalogger.log("a_str", "str")
        datalogger.log("a_float", 2.0)
        datalogger.log("an_int", 2)
        datalogger.log("an_int_array", np.array([2, 2, 2, 2]))
        time.sleep(1)

except KeyboardInterrupt:
    rospy.loginfo(rospy.get_name()+"::Keyboard interrupt, killing node.")
