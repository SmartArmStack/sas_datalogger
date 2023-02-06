#!/usr/bin/python3

# Copyright (c) 2012-2021 Murilo Marques Marinho
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
# ################################################################*/

##############################################################################
#                                INCLUDES
##############################################################################
import datetime

import rospy

# Scipy imports
import scipy.io as sio

# Services
from sas_datalogger.srv import Save
from sas_datalogger.srv import SaveResponse

from sas_datalogger.msg import AddValueMsg


##############################################################################
#                            NODE MAIN ROUTINE
##############################################################################


def main():
    # Initialize Node
    rospy.init_node('sas_datalogger_node', disable_signals=True)

    # Initialize object
    sas_datalogger = RosiloDatalogger()

    try:
        # Create main loop
        rospy.spin()
    except KeyboardInterrupt:
        pass

    filename = '{date:%Y_%m_%d_%H_%M_%S}.mat'.format(date=datetime.datetime.now())
    print("sas_datalogger_node::Saving datalog to filename = '{}'.".format(filename))
    sas_datalogger.save(filename)


##############################################################################
#                            CLASS RosiloDatalogger
##############################################################################

class RosiloDatalogger:

    ###########################################################################
    #                         CONSTRUCTORS AND DESTRUCTORS
    ###########################################################################

    def __init__(self):

        # Setting status variables
        self.data = {}

        # Initialize service servers
        self.service_server_save = rospy.Service('/sas_datalogger/save', Save, self.SaveCallback)

        # Initialize subscribers
        self.subscriber_addvalue = rospy.Subscriber("/sas_datalogger/addvaluemsg", AddValueMsg,
                                                    self.AddValueMsgCallback)

    def save(self, filename):
        # Save
        sio.savemat(filename, self.data)

        # Clear buffer
        self.data = {}

    ###########################################################################
    #                     SERVICE SERVER CALLBACK FUNCTIONS
    ###########################################################################

    def SaveCallback(self, req):
        self.save(req.filename)
        return SaveResponse(True)

    ###########################################################################
    #                     SUBSCRIBER CALLBACK FUNCTIONS
    ###########################################################################
    # ADDVALUE CALLBACK
    def AddValueMsgCallback(self, req):

        # Initialize list for a given variable
        if req.name in self.data:
            pass
        else:
            self.data[req.name] = []

        # Append value to dictionary, string or not
        if len(req.value) > 0:
            self.data[req.name].append(req.value)
        else:
            self.data[req.name].append(req.strvalue)

        return


##############################################################################
#                          RUNNING THE MAIN ROUTINE
##############################################################################


if __name__ == '__main__':
    main()
