#!/usr/bin/python3
import rospy
import time
import numpy as np
from sas_datalogger import DataloggerInterface

rospy.init_node('sas_datalogger_interface_test', disable_signals=True)

datalogger = DataloggerInterface(20)
while not rospy.is_shutdown():
    datalogger.log("a_str", "str")
    datalogger.log("a_float", 2.0)
    datalogger.log("an_int", 2)
    datalogger.log("an_int_array", np.array([2, 2, 2, 2]))
    time.sleep(1)
