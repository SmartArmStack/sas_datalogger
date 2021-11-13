/*
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

#include "sas_datalogger/sas_datalogger_interface.h"

namespace sas
{

DataloggerInterface::DataloggerInterface(ros::NodeHandle& node_handle, int queue_size)
{
    sc_save         = node_handle.serviceClient<sas_datalogger::Save>("/sas_datalogger/save");
    pub_add_value   = node_handle.advertise<sas_datalogger::AddValueMsg>("/sas_datalogger/addvaluemsg",queue_size);
}

void DataloggerInterface::log(const std::string& name, const Eigen::VectorXd& value)
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

void DataloggerInterface::log(const std::string& name, const std::vector<double>& value)
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

void DataloggerInterface::log(const std::string& name, const double& value)
{
    pub_add_value_msg.value.resize(1);
    pub_add_value_msg.value[0] = value;
    pub_add_value_msg.strvalue = std::string("");

    pub_add_value_msg.name  = name;

    pub_add_value.publish( pub_add_value_msg );
}

void DataloggerInterface::log(const std::string& name, const std::string& value)
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
