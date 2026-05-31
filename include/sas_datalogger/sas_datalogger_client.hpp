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

/**
 * @brief Lightweight client for publishing log data.
 *
 * DataloggerClient provides helpers to publish various types (matrices,
 * vectors, scalars and strings) as `sas_msgs::msg::LogDatum` messages.
 * It is intended for runtime logging and can also request saving logged
 * data to a file via the `save` method.
 */
class DataloggerClient: private sas::Object
{
private:
    //ros::ServiceClient sc_save;
    //sas_datalogger::Save     sm_save;

    rclcpp::Publisher<sas_msgs::msg::LogDatum>::SharedPtr publisher_log_;
public:
    /**
     * @brief Construct a new DataloggerClient.
     *
     * @param node Shared pointer to the ROS2 node used for publishing.
     * @param queue_size Publisher queue size (default: 100).
     */
    DataloggerClient(const rclcpp::Node::SharedPtr& node, const size_t &queue_size=100);

    /**
     * @brief Check if the datalogger is enabled (publisher is valid).
     *
     * @return true when enabled and ready to publish messages.
     * @return false otherwise.
     */
    bool is_enabled() const;

    /**
     * @brief Log a matrix value under the given name.
     *
     * @param name Key/name under which the value will be logged.
     * @param value The Eigen::MatrixXd value to log.
     */
    void log(const std::string& name, const MatrixXd& value);

    /**
     * @brief Log a vector value under the given name.
     *
     * @param name Key/name under which the value will be logged.
     * @param value The Eigen::VectorXd value to log.
     */
    void log(const std::string& name, const VectorXd& value);

    /**
     * @brief Log a std::vector<double> value under the given name.
     *
     * @param name Key/name under which the value will be logged.
     * @param value The std::vector<double> value to log.
     */
    void log(const std::string& name, const std::vector<double>& value);

    /**
     * @brief Log a scalar double value under the given name.
     *
     * @param name Key/name under which the value will be logged.
     * @param value The double value to log.
     */
    void log(const std::string& name, const double& value);

    /**
     * @brief Log a string value under the given name.
     *
     * @param name Key/name under which the value will be logged.
     * @param value The string value to log.
     */
    void log(const std::string& name, const std::string& value);

    /**
     * @brief Request saving the logged data to the specified filename.
     *
     * @param filename Path to the file where data should be saved.
     */
    void save(const std::string &filename);
};

}

