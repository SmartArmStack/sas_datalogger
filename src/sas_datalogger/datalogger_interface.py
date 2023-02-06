"""
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
# ################################################################
"""
import rospy
from sas_datalogger.msg import AddValueMsg
from std_msgs.msg import Float64MultiArray


class DataloggerInterface:
    def __init__(self, queue_size):
        # sc_save = node_handle.serviceClient < sas_datalogger::Save > ("/sas_datalogger/save");
        self.publisher_add_value_ = rospy.Publisher("/sas_datalogger/addvaluemsg",
                                                    AddValueMsg,
                                                    queue_size=queue_size)

    def log(self, name, value):
        msg = AddValueMsg()
        msg.name = name
        if type(value) is str:
            msg.strvalue = value
            self.publisher_add_value_.publish(msg)
            return

        elif type(value) is float or type(value) is int:
            msg.value = [value]
            self.publisher_add_value_.publish(msg)
            return

        else:
            # If the array is (x, 1)
            # If the array is (1, y)
            try:
                (x, y) = value.shape
                msg.value = value.reshape(max(x, y))
                self.publisher_add_value_.publish(msg)
                return

            except ValueError as e:
                pass
            # If the array is (x,)
            try:
                (x,) = value.shape
                msg.value = value
                self.publisher_add_value_.publish(msg)
                return

            except ValueError as e:
                pass

        raise Exception("DataloggerInterface::Unknown value type={}.".format(type(value)))
