#!/bin/bash
set -e
sudo apt-get update && sudo apt-get upgrade -y
sudo apt-get install -y libxcb-cursor-dev
python3 -m pip install PyQt6 pyqtgraph --break-system-packages
# The apt scipy is built for numpy 1, so it fails to import ("numpy.core.multiarray failed to import")
# under the pip numpy 2 that dqrobotics brings; sas_datalogger saves with scipy.io. Install a scipy
# built for numpy 2 over it (pip cannot uninstall Debian's, and numpy is already there).
python3 -m pip install "scipy>=1.14" --ignore-installed --no-deps --break-system-packages
# https://github.com/bhowiebkr/laser-level-webcam/issues/28
python3 -m pip install pyqtdarktheme==2.1.0 --ignore-requires-python --break-system-packages
