#pragma once
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

#include <ros/ros.h>

#include <sas_datalogger/Save.h>
#include <sas_datalogger/AddValueMsg.h>

//Eigen
#include <eigen3/Eigen/Dense>

namespace sas
{

class DataloggerInterface
{
private:
    ros::ServiceClient sc_save;
    sas_datalogger::Save     sm_save;

    ros::Publisher     pub_add_value;
    sas_datalogger::AddValueMsg pub_add_value_msg;
public:
    DataloggerInterface(ros::NodeHandle& node_handle, int queue_size);

    void log(const std::string& name, const Eigen::VectorXd& value);
    void log(const std::string& name, const std::vector<double>& value);
    void log(const std::string& name, const double& value);
    void log(const std::string& name, const std::string& value);

    void save(const std::string &filename);
};

}

