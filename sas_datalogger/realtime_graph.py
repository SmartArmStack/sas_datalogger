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
"""Realtime plotting class used by sas_datalogger GUI components.

Provides a convenience wrapper around ``pyqtgraph`` to display a
streaming scalar signal with an internal fixed-size queue.
"""

import numpy as np
from queue import Queue
from PyQt6.QtWidgets import QWidget
import pyqtgraph as pg

class RealtimeGraph:
    """Realtime plotting class.

    Maintains a fixed-size queue of recent scalar samples and updates a
    ``pyqtgraph`` window with the latest data and automatic Y-range.
    """

    def __init__(self,
                 title: str,
                 lims: tuple[float, float]=[0,0],
                 max_queue_size: int=200):
        """Create the realtime plot.

        Args:
            title: Window title for the plot.
            lims: Initial Y-axis limits as (min, max).
            max_queue_size: Maximum number of samples to retain.
        """

        self.plot = pg.plot(title=title)
        self.title = title
        self.queue = Queue(maxsize=max_queue_size)
        self.lims = list(lims)
        self.plot_data = self.plot.plot([])

    def update(self, datum: float):
        """Append a new scalar sample and refresh the plot.

        Args:
            datum: Scalar value to append to the internal buffer.
        """

        if self.queue.full():
            self.queue.get()
        self.queue.put(datum)

        current_data = np.asarray(self.queue.queue)

        self.lims[0] = min(self.lims[0], np.min(current_data))
        self.lims[1] = max(self.lims[1], np.max(current_data))
        self.plot_data.setData(np.linspace(0, 1, self.queue.qsize()), current_data)

        self.plot.setYRange(self.lims[0], self.lims[1])
        self.plot.setTitle(self.title)

