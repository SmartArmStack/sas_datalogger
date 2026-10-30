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
"""Round-trip tests between the DataloggerClient and the SASDatalogger node.

The C++ client publishes sas_msgs/LogDatum messages on /sas_datalogger/log;
the Python SASDatalogger subscribes and accumulates them, and can persist
the result to a MATLAB file via save().
"""

import time

import numpy as np
import pytest
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.utilities import ok, try_shutdown
from scipy.io import loadmat

from sas_common import rclcpp_init, rclcpp_Node, rclcpp_shutdown, rclcpp_spin_some
from sas_datalogger import DataloggerClient
from sas_datalogger.sas_datalogger import SASDatalogger

TIMEOUT = 15.0


@pytest.fixture(scope="module")
def rclpy_context():
    """Initialize the ROS context once for the whole module."""
    if not ok():
        rclpy.init()
    try:
        rclcpp_init()
        cpp_initialized = True
    except RuntimeError:
        cpp_initialized = False
    yield
    if cpp_initialized:
        rclcpp_shutdown()
    try_shutdown()


def _pump(client_node, datalogger):
    rclcpp_spin_some(client_node)
    try:
        rclpy.spin_once(datalogger, timeout_sec=0.0)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception:
        pass


def _wait_until(predicate, client_node, datalogger, timeout=TIMEOUT):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        _pump(client_node, datalogger)
        if predicate():
            return True
        time.sleep(0.005)
    return predicate()


@pytest.fixture()
def client_server_pair(rclpy_context):
    """Yield a connected (DataloggerClient, SASDatalogger) pair."""
    client_node = rclcpp_Node("datalogger_client_node")
    datalogger = SASDatalogger(node_name="datalogger_server_node")
    client = DataloggerClient(client_node)
    assert _wait_until(client.is_enabled, client_node, datalogger)
    try:
        yield client, datalogger, client_node
    finally:
        datalogger.destroy_node()


def _sample(datalogger, key):
    """Return the most recent logged sample for key as floats, or None."""
    samples = datalogger.data.get(key)
    if not samples:
        return None
    return [float(v) for v in np.asarray(samples[-1]).ravel()]


def test_log_scalar(client_server_pair):
    client, datalogger, client_node = client_server_pair
    def logged():
        client.log("scalar", 3.5)
        return _sample(datalogger, "scalar") == [3.5]

    assert _wait_until(logged, client_node, datalogger)


def test_log_vector(client_server_pair):
    client, datalogger, client_node = client_server_pair
    def logged():
        client.log("vector", [1.0, 2.0, 3.0])
        return _sample(datalogger, "vector") == [1.0, 2.0, 3.0]

    assert _wait_until(logged, client_node, datalogger)


def test_log_matrix(client_server_pair):
    client, datalogger, client_node = client_server_pair
    matrix = [[1.0, 2.0], [3.0, 4.0]]
    def logged():
        client.log("matrix", matrix)
        stored = datalogger.data.get("matrix")
        # The C++ client packs Eigen column-major, so the stored sample is
        # the transpose of the row-major input.
        return bool(stored) and np.allclose(
            np.asarray(stored[-1]), np.asarray(matrix).T)

    assert _wait_until(logged, client_node, datalogger)


def test_log_string(client_server_pair):
    client, datalogger, client_node = client_server_pair
    def logged():
        client.log("label", "hello")
        samples = datalogger.data.get("label")
        return bool(samples) and samples[-1] == "hello"

    assert _wait_until(logged, client_node, datalogger)


def test_multiple_updates_accumulate(client_server_pair):
    client, datalogger, client_node = client_server_pair
    tail = []
    total = 0

    def five_samples():
        nonlocal tail, total
        for i in range(5):
            client.log("counter", float(i))
            # Pump between writes: back-to-back bursts get dropped by the
            # DDS layer before the subscriber is drained.
            _pump(client_node, datalogger)
        samples = datalogger.data.get("counter", [])
        total = len(samples)
        tail = [float(np.asarray(s).ravel()[0]) for s in samples[-5:]]
        # DDS gives no arrival-order guarantee across samples, so compare
        # the window as a multiset.
        return total >= 5 and sorted(tail) == [0.0, 1.0, 2.0, 3.0, 4.0]

    assert _wait_until(five_samples, client_node, datalogger, timeout=30.0), (
        f"tail={tail} of {total} samples")


def test_save_persists_mat_file(client_server_pair, tmp_path):
    client, datalogger, client_node = client_server_pair
    def logged():
        client.log("saved", 42.0)
        return _sample(datalogger, "saved") == [42.0]

    assert _wait_until(logged, client_node, datalogger)
    filename = str(tmp_path / "datalogger_test.mat")
    datalogger.save(filename)
    loaded = loadmat(filename)
    assert np.allclose(loaded["saved"].ravel(), [42.0])
    assert datalogger.data == {}
