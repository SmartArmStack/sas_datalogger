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
#   Author: Murilo M. Marinho, email: murilomarinho@ieee.org
#
# ################################################################*/

#include <rclcpp/rclcpp.hpp>

#include <sas_core/sas_object.hpp>
#include <sas_msgs/msg/log_datum.hpp>

#include <eigen3/Eigen/Dense>

using namespace Eigen;

namespace sas
{

class DataloggerClient: private sas::Object
{
private:
    //ros::ServiceClient sc_save;
    //sas_datalogger::Save     sm_save;

    rclcpp::Publisher<sas_msgs::msg::LogDatum>::SharedPtr publisher_log_;
public:
    DataloggerClient(const rclcpp::Node::SharedPtr& node, const size_t &queue_size=100);

    bool is_enabled() const;

    void log(const std::string& name, const MatrixXd& value);
    void log(const std::string& name, const VectorXd& value);
    void log(const std::string& name, const std::vector<double>& value);
    void log(const std::string& name, const double& value);
    void log(const std::string& name, const std::string& value);

    void save(const std::string &filename);
};

}

