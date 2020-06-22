#!/usr/bin/python3

# Copyright (c) 2012-2020 Murilo Marques Marinho
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met
#
#     - Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     - Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     - Neither the name of the Automation and Robotics Lab (LARA) nor the
#       names of its contributors may be used to endorse or promote products
#       derived from this software without specific prior written permission.
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU Lesser General Public License LGPL as
# published by the Free Software Foundation, either version 3 of the
# License, or (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
# GNU Lesser General Public License LGPL for more details.
#
# You should have received a copy of the GNU Lesser General Public
# License LGPL along with this program.
# If not, see <http://www.gnu.org/licenses/>.
#
# ################################################################
#
#   Author: Murilo M. Marinho, email: murilo@nml.t.u-tokyo.ac.jp
#
# ################################################################


##############################################################################
#                                INCLUDES
##############################################################################
import rospy

# Scipy imports
import scipy.io as sio

# Services
from rosilo_datalogger.srv import Save
from rosilo_datalogger.srv import SaveResponse

from rosilo_datalogger.msg import AddValueMsg

##############################################################################
#                            NODE MAIN ROUTINE
##############################################################################


def main():

    # Initialize Node
    rospy.init_node('rosilo_datalogger')

    # Initialize object
    rosilo_datalogger = RosiloDatalogger()

    # Create main loop
    rospy.spin()

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
#                            CLASS UnbPlot
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
        self.service_server_save = rospy.Service('/rosilo_datalogger/save', Save, self.SaveCallback )

        # Initialize subscribers
        self.subscriber_addvalue = rospy.Subscriber("/rosilo_datalogger/addvaluemsg", AddValueMsg, self.AddValueMsgCallback)

    ###########################################################################
    #                     SERVICE SERVER CALLBACK FUNCTIONS
    ###########################################################################

    def SaveCallback(self, req):

        # Create dictionary
        data = {}

        # Add values
        for variable in self.variables:
            data[str(variable.name)] = variable.values

        # Save
        sio.savemat(req.filename, data)

        rospy.loginfo('RosiloDatalogger::SaveCallback - Data saved in file ' + str(req.filename))

        self.variables = []  # Clear buffers
        self.currentvariablenames = []  # Clear buffers
        self.numberofvariables = 0

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
    try:
        main()
    except rospy.ROSInterruptException:
        pass
