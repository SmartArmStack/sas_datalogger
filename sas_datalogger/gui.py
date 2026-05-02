"""
Copyright (C) 2020-2026 Murilo Marques Marinho (www.murilomarinho.info)
This program is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later
version.
This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied
warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
You should have received a copy of the GNU General Public License along with this program. If not,
see <https://www.gnu.org/licenses/>.
"""
import rclpy
from rclpy.node import Node

from PyQt6.QtCore import QTimer
from PyQt6.QtWidgets import QWidget, QApplication, QMainWindow, QHBoxLayout

import qdarktheme
import pyqtgraph as pg

class DataloggerWindow(QMainWindow):
    def __init__(self):
        super().__init__()

        self.timer_ = QTimer()
        self.timer_.timeout.connect(self._timer_callback)
        self.timer_.start(1)

        self.central_widget = QWidget()
        self.layout = QHBoxLayout(self)
        self.central_widget.setLayout(self.layout)
        self.setCentralWidget(self.central_widget)

    def _timer_callback(self):
        try:
            pass
        except ...:
            pass

def main(args=None):
    rclpy.init()
    app = QApplication([])
    myapp = DataloggerWindow()
    qdarktheme.setup_theme()
    myapp.show()
    app.exec()

if __name__ == "__main__":
    main()
