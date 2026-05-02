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
import rclpy
from sas_datalogger.sas_datalogger import SASDatalogger


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

if __name__ == '__main__':
    main()
