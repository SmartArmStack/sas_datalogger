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

"""GUI widgets for the SAS datalogger package.

This module provides a vertical slider widget with descriptive and
value labels used by the datalogger UI.
"""

from PyQt6.QtCore import Qt
from PyQt6.QtWidgets import QWidget, QSlider, QVBoxLayout, QLabel


class SliderLabelVertical(QWidget):
    """Widget combining a vertical QSlider with descriptive and value labels.

    The widget shows a description label, a label that displays the current
    value as text, and a vertical slider. It is intended for control and
    display in GUI panels.
    """

    def __init__(self,
                 label:str,
                 slider_range:tuple[int,int],
                 parent=None):
        """Initialize the widget.

        Args:
            label: Descriptive text shown above the slider.
            slider_range: Tuple (min, max) describing the integer range.
            parent: Optional parent widget.
        """
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
        """Set the slider's numeric value.

        Args:
            value: Integer value to set the slider to.
        """
        self.slider.setValue(value)

    def set_text(self, text):
        """Set the text displayed in the value label.

        Args:
            text: Text to display next to the slider.
        """
        self.value_label.setText(text)