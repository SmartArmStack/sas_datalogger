/*
# Copyright (c) 2012-2023 Murilo Marques Marinho
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

#include <sas_datalogger/sas_datalogger_client.hpp>

namespace sas
{

DataloggerClient::DataloggerClient(const rclcpp::Node::SharedPtr &node, int queue_size):
    sas::Object("sas::DataloggerClient")
{
    //sc_save         = node.serviceClient<sas_datalogger::Save>("/sas_datalogger/save");
    //pub_add_value   = node.advertise<sas_datalogger::AddValueMsg>("/sas_datalogger/addvaluemsg",queue_size);
    publisher_log_ = node->create_publisher<sas_msgs::msg::LogDatum>("/sas_datalogger/log",queue_size);
}

bool DataloggerClient::is_enabled() const
{
    return (publisher_log_->get_subscription_count() > 0);
}

void DataloggerClient::log(const std::string &name, const MatrixXd &value)
{
    sas_msgs::msg::LogDatum msg;

    msg.name = name;
    msg.value = std::vector<double>(value.data(), value.data() + value.rows() * value.cols());
    msg.layout = {int(value.rows()),int(value.cols())};
    msg.strvalue = std::string("");

    publisher_log_->publish(msg);
}

void DataloggerClient::log(const std::string& name, const Eigen::VectorXd& value)
{
    sas_msgs::msg::LogDatum msg;

    msg.name  = name;
    msg.value = std::vector<double>(value.data(), value.data() + value.rows() * value.cols());
    msg.strvalue = std::string("");

    publisher_log_->publish(msg);
}

void DataloggerClient::log(const std::string& name, const std::vector<double>& value)
{
    sas_msgs::msg::LogDatum msg;

    msg.name  = name;
    msg.value = value;
    msg.strvalue = std::string("");

    publisher_log_->publish(msg);
}

void DataloggerClient::log(const std::string& name, const double& value)
{
    sas_msgs::msg::LogDatum msg;

    msg.value.resize(1);
    msg.value[0] = value;
    msg.strvalue = std::string("");

    msg.name  = name;

    publisher_log_->publish(msg);
}

void DataloggerClient::log(const std::string& name, const std::string& value)
{
    sas_msgs::msg::LogDatum msg;

    msg.value.resize(0);
    msg.strvalue = value;
    msg.name = name;

    publisher_log_->publish(msg);
}


//void DataloggerClient::save(const std::string& filename)
//{
//    sm_save.request.filename = filename;/
//
//    sc_save.call( sm_save );
//}

}
