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
import signal
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions

from sas_datalogger.realtime_graph import RealtimeGraph
from sas_datalogger.sas_datalogger import SASDatalogger

from PyQt6.QtCore import QTimer
from PyQt6.QtWidgets import QWidget, QApplication, QMainWindow, QHBoxLayout

import qdarktheme
import pyqtgraph as pg

def sigint_handler(*args):
    QApplication.quit()
    # rclpy.shutdown()

class DataloggerWindow(QMainWindow):
    def __init__(self,
                 sampling_time: float = 0.001,
                 parent = None):
        super().__init__(parent)

        self.sampling_time = sampling_time

        self.datalogger = SASDatalogger(node_name="sas_datalogger_gui_node")

        self.timer_ = QTimer()
        self.timer_.timeout.connect(self._timer_callback)
        self.timer_.start(int(sampling_time * 1000.0))

        self.realtime_graphs_dict: dict = dict()

        self.central_widget = QWidget()
        self.layout = QHBoxLayout()
        self.central_widget.setLayout(self.layout)
        self.setCentralWidget(self.central_widget)

    def _timer_callback(self):
        try:
            rclpy.spin_once(self.datalogger, timeout_sec=self.sampling_time)

            for key, value in self.datalogger.data.items():

                if isinstance(value[-1], np.ndarray):
                    datum = np.squeeze(value)[-1]
                    if len(datum.shape) > 1:
                        continue # Ignore matrices, no real way to plot them.

                datum = value[-1]

                if isinstance(datum, str):
                    continue # Ignore strings, no real way to plot them.

                if len(datum) > 1:
                    continue # TODO: Handle multiline plots.

                datum = datum[-1]  # It's received as a pair, for instance ('d', 5.0)

                if key in self.realtime_graphs_dict:
                    self.realtime_graphs_dict[key].update(datum)
                else:
                    print(f"Creating plot for: {key}. Valid datum = {datum}")
                    self.realtime_graphs_dict[key] = RealtimeGraph(key)
                    self.layout.addWidget(self.realtime_graphs_dict[key].plot)
                    self.realtime_graphs_dict[key].update(datum)
        except Exception as e:
            print(e)


def main(args=None):
    signal.signal(signal.SIGINT, sigint_handler)
    try:
        rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
        app = QApplication([])
        myapp = DataloggerWindow()
        qdarktheme.setup_theme()
        myapp.show()
        app.exec()
    except KeyboardInterrupt:
        pass

if __name__ == "__main__":
    main()
