#!/usr/bin/python3

# Copyright (c) 2012-2020 Murilo Marques Marinho
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
#   Author: Murilo M. Marinho, email: murilo@nml.t.u-tokyo.ac.jp
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
#                            CLASS Plot Dataholder
##############################################################################


class Variable:

    def __init__(self, name, vartype, size):

        self.name = name
        self.vartype = vartype
        self.size = size
        self.values = []

##############################################################################
#                            CLASS RosiloDatalogger
##############################################################################


class RosiloDatalogger:

    ###########################################################################
    #                         CONSTRUCTORS AND DESTRUCTORS
    ###########################################################################

    def __init__(self):

        # Setting status variables
        self.variables = []  # A list of variables
        self.currentvariablenames = []
        self.numberofvariables = 0

        # Initialize service servers
        self.service_server_save = rospy.Service('/sas_datalogger/save', Save, self.SaveCallback )

        # Initialize subscribers
        self.subscriber_addvalue = rospy.Subscriber("/sas_datalogger/addvaluemsg", AddValueMsg, self.AddValueMsgCallback)

    def save(self, filename):
        # Create dictionary
        data = {}

        # Add values
        for variable in self.variables:
            data[str(variable.name)] = variable.values

        # Save
        sio.savemat(filename, data)

        self.variables = []  # Clear buffers
        self.currentvariablenames = []  # Clear buffers
        self.numberofvariables = 0

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

        # Check if name is within existing variables
        if not (req.name in self.currentvariablenames):
            self.numberofvariables = self.numberofvariables+1

            variable = Variable(req.name, str("float"), len(req.value))
            self.variables.append(variable)

            # Store the existing variable names
            self.currentvariablenames = set(variable.name for variable in self.variables)

        # Search list for the right object
        for x in self.variables:
            if x.name == req.name:
                variable = x
                break

        if len(req.value) > 0:
            # Update data values
            variable.values.append(req.value)
        else:
            variable.values.append(req.strvalue)

        return

##############################################################################
#                          RUNNING THE MAIN ROUTINE
##############################################################################


if __name__ == '__main__':
    main()
