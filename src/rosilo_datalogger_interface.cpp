/*
# Copyright (c) 2012-2020 Murilo Marques Marinho
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#     - Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     - Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     - Neither the name of the Mitsuishi Sugita Laboratory (NML) nor the
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
# ################################################################*/

#include "rosilo_datalogger/rosilo_datalogger_interface.h"

namespace rosilo
{

DataloggerInterface::DataloggerInterface(ros::NodeHandle& node_handle, int queue_size)
{
    sc_save         = node_handle.serviceClient<rosilo_datalogger::Save>("/rosilo_datalogger/save");
    pub_add_value   = node_handle.advertise<rosilo_datalogger::AddValueMsg>("/rosilo_datalogger/addvaluemsg",queue_size);
}

void DataloggerInterface::addValueMsg(const std::string& name, const Eigen::VectorXd& value)
{
    pub_add_value_msg.value.resize(value.size());

    for(unsigned int i=0;i < value.size();i++)
    {
        pub_add_value_msg.value[i] = value(i);
    }

    pub_add_value_msg.name  = name;
    pub_add_value_msg.strvalue = std::string("");

    pub_add_value.publish( pub_add_value_msg );
}

void DataloggerInterface::addValueMsg(const std::string& name, const std::vector<double>& value)
{
    pub_add_value_msg.value.resize(value.size());

    for(unsigned int i=0;i < value.size();i++)
    {
        pub_add_value_msg.value[i] = value[i];
    }

    pub_add_value_msg.name  = name;
    pub_add_value_msg.strvalue = std::string("");

    pub_add_value.publish( pub_add_value_msg );
}

void DataloggerInterface::addValueMsg(const std::string& name, const double& value)
{
    pub_add_value_msg.value.resize(1);
    pub_add_value_msg.value[0] = value;
    pub_add_value_msg.strvalue = std::string("");

    pub_add_value_msg.name  = name;

    pub_add_value.publish( pub_add_value_msg );
}

void DataloggerInterface::addValueMsg(const std::string& name, const std::string& value)
{
    pub_add_value_msg.value.resize(0);
    pub_add_value_msg.strvalue = value;
    pub_add_value_msg.name = name;
    pub_add_value.publish( pub_add_value_msg );
}


void DataloggerInterface::save(const std::string& filename)
{
    sm_save.request.filename = filename;

    sc_save.call( sm_save );
}

}
