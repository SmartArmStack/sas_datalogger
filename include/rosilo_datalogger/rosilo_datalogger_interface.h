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

#ifndef UNB_PLOTTER_HEADER_GUARD
#define UNB_PLOTTER_HEADER_GUARD

#include <ros/ros.h>

#include <rosilo_datalogger/Save.h>
#include <rosilo_datalogger/AddValueMsg.h>

//Eigen
#include <eigen3/Eigen/Dense>

class RosiloDataloggerInterface
{
private:
    ros::ServiceClient sc_save;
    rosilo_datalogger::Save     sm_save;

    ros::Publisher     pub_add_value;
    rosilo_datalogger::AddValueMsg pub_add_value_msg;
public:
    RosiloDataloggerInterface(ros::NodeHandle& node_handle, int queue_size);

    void addValueMsg(const std::string& name, const Eigen::VectorXd& value);
    void addValueMsg(const std::string& name, const std::vector<double>& value);
    void addValueMsg(const std::string& name, const double& value);
    void addValueMsg(const std::string& name, const std::string& value);

    //Alias for addValueMsg
    void log(const std::string& name, const Eigen::VectorXd& value)
    {addValueMsg(name,value);}
    void log(const std::string& name, const std::vector<double>& value)
    {addValueMsg(name,value);}
    void log(const std::string& name, const double& value)
    {addValueMsg(name,value);}
    void log(const std::string& name, const std::string& value)
    {addValueMsg(name,value);}

    void save(const std::string &filename);
};

#endif
