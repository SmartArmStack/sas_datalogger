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
    rclpy.shutdown()

class DataloggerWindow(QMainWindow):
    def __init__(self):
        super().__init__()

        self.datalogger = SASDatalogger(node_name="sas_datalogger_gui_node")
        self.timer_ = QTimer()
        self.timer_.timeout.connect(self._timer_callback)
        self.timer_.start(1)

        self.central_widget = QWidget()
        self.layout = QHBoxLayout(self)
        self.central_widget.setLayout(self.layout)
        self.setCentralWidget(self.central_widget)

    def _timer_callback(self):
        try:
            rclpy.spin_once(self.datalogger)
        except ...:
            pass

def main(args=None):
    signal.signal(signal.SIGINT, sigint_handler)
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    app = QApplication([])
    myapp = DataloggerWindow()
    qdarktheme.setup_theme()
    myapp.show()
    app.exec()

if __name__ == "__main__":
    main()
