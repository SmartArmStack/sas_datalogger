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
from math import sqrt
import signal
import numpy as np
import threading

import rclpy
from rclpy.signals import SignalHandlerOptions

from rcl_interfaces.msg import ParameterType, ParameterDescriptor

from sas_datalogger.realtime_graph import RealtimeGraph
from sas_datalogger.sas_datalogger import SASDatalogger

from PyQt6.QtCore import QTimer
from PyQt6.QtWidgets import QWidget, QApplication, QMainWindow, QHBoxLayout, QGridLayout

import qdarktheme
import pyqtgraph as pg

def sigint_handler(*args):
    QApplication.quit()
    rclpy.shutdown()

class DataloggerWindow(QMainWindow):
    def __init__(self,
                 sampling_time: float = 0.001,
                 parent = None):
        super().__init__(parent)

        self.sampling_time = sampling_time

        self.datalogger = SASDatalogger(node_name="sas_datalogger_gui_node")
        self.datalogger_spin_thread = threading.Thread(target=self.spin)

        # rclpy issue 912
        self.datalogger.declare_parameter(
            'whitelist',
            [""],
            ParameterDescriptor
            (
                type=ParameterType.PARAMETER_STRING_ARRAY
            ))
        self.whitelist: list[str] | None = self.datalogger.get_parameter('whitelist').get_parameter_value().string_array_value
        if self.whitelist == [' ']:
            self.whitelist = None
            print(f"Whitelist empty, plotting all values in datalogger topic.")
        else:
            print(f"Whitelist: {self.whitelist}")

        self.timer_ = QTimer()
        self.timer_.timeout.connect(self._timer_callback)
        self.timer_.start(int(sampling_time * 1000.0))

        self.realtime_graphs_dict: dict = dict()

        self.central_widget = QWidget()
        if self.whitelist is None:
            self.layout = QHBoxLayout()
        else:
            self.layout = QGridLayout()
        self.central_widget.setLayout(self.layout)
        self.setCentralWidget(self.central_widget)

        self.datalogger_spin_thread.start()

    def spin(self):
        rclpy.spin(self.datalogger)

    def _timer_callback(self):
        try:
            # rclpy.spin_once(self.datalogger, timeout_sec=self.sampling_time)

            for key, value in self.datalogger.data.items():

                if self.whitelist is not None:
                    if key not in self.whitelist:
                        continue

                if isinstance(value[-1], np.ndarray):
                    datum = np.squeeze(value)[-1]
                    if len(datum.shape) > 1:
                        continue # Ignore matrices, no real way to plot them.

                datum = value[-1]

                if isinstance(datum, str):
                    continue # Ignore strings, no real way to plot them.

                if len(datum) > 1:
                    datum = np.squeeze(datum)
                else:
                    datum = datum[-1]  # It's received as a pair, for instance ('d', 5.0)

                if key in self.realtime_graphs_dict:
                    self.realtime_graphs_dict[key].update(datum)
                else:
                    # print(f"Creating plot for: {key}. Valid datum = {datum}")
                    self.realtime_graphs_dict[key] = RealtimeGraph(key)
                    if self.whitelist is None:
                        self.layout.addWidget(self.realtime_graphs_dict[key].plot)
                    else:
                        count = len(self.realtime_graphs_dict)
                        total = len(self.whitelist)
                        # 16:9 aspect ratio
                        row_max = int(total**(16.0/25.0))
                        row = int(count/row_max)
                        col = count%row_max
                        self.layout.addWidget(self.realtime_graphs_dict[key].plot,row,col)
                    self.realtime_graphs_dict[key].update(datum)
        except Exception as e:
            print(e)


def main(args=None):
    signal.signal(signal.SIGINT, sigint_handler)
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    try:

        app = QApplication([])
        myapp = DataloggerWindow()
        qdarktheme.setup_theme()
        myapp.show()
        app.exec()
    except KeyboardInterrupt:
        pass

if __name__ == "__main__":
    main()
