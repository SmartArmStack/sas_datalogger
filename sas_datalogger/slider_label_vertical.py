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

from PyQt6.QtCore import Qt
from PyQt6.QtWidgets import QWidget, QSlider, QVBoxLayout, QLabel


class SliderLabelVertical(QWidget):
    def __init__(self,
                 label:str,
                 slider_range:tuple[int,int],
                 parent=None):
        super().__init__(parent)

        self.description_label = QLabel()
        self.description_label.setText(label)

        self.value_label = QLabel()
        self.value_label.setText(label)

        self.slider = QSlider(Qt.Orientation.Vertical)
        self.slider.setRange(slider_range[0], slider_range[1])

        self.layout = QVBoxLayout()
        self.layout.addWidget(self.description_label)
        self.layout.addWidget(self.value_label)
        self.layout.addWidget(self.slider)

        self.setLayout(self.layout)

    def set_value(self, value):
        self.slider.setValue(value)

    def set_text(self, text):
        self.value_label.setText(text)