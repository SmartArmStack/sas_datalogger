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

#include <rclcpp/rclcpp.hpp>
#include <sas_common/sas_common.hpp>
#include <sas_datalogger/sas_datalogger_client.hpp>

/*********************************************
 * SIGNAL HANDLER
 * *******************************************/
#include<signal.h>
static std::atomic_bool kill_this_process(false);
void sig_int_handler(int)
{
    kill_this_process = true;
}


int main(int argc,char** argv)
{
    if(signal(SIGINT, sig_int_handler) == SIG_ERR)
    {
        throw std::runtime_error("::Error setting the signal int handler.");
    }

    rclcpp::init(argc,argv,rclcpp::InitOptions(),rclcpp::SignalHandlerOptions::None);

    auto node = std::make_shared<rclcpp::Node>("sas_datalogger_client_example");

    try
    {
        sas::DataloggerClient datalogger_client(node); //If you're storing more than 100 values at each loop, this might need adjustment

        RCLCPP_WARN_STREAM_ONCE(node->get_logger(), "*******RUN THIS EXAMPLE WITH THE LAUNCH FILE, NOT THE BINARY DIRECTLY.*******");
        RCLCPP_WARN_STREAM_ONCE(node->get_logger(), "Run with 'ros2 launch sas_datalogger sas_datalogger_client_example_launch.py'");
        RCLCPP_WARN_STREAM_ONCE(node->get_logger(), "*****************************************************************************");

        RCLCPP_INFO_STREAM_ONCE(node->get_logger(), "Waiting for a connection with sas_datalogger...");
        while( (not datalogger_client.is_enabled()) and (not kill_this_process))
        {
            std::this_thread::sleep_for(std::chrono::microseconds(100));
            rclcpp::spin_some(node);
        }
        RCLCPP_INFO_STREAM_ONCE(node->get_logger(), "Connected to sas_datalogger.");

        RCLCPP_INFO_STREAM_ONCE(node->get_logger(), "Logging started...");

        RCLCPP_INFO_STREAM_ONCE(node->get_logger(), "Logging each value 5 times...");

        for (auto i = 0; i < 5; i++)
        {
            MatrixXd A(3,4); A << 1,2,3,4,5,6,7,8,9,10,11,12;
            datalogger_client.log("A",A);

            VectorXd v(5); v << 1,5,10,15,20;
            datalogger_client.log("v",v);

            std::vector<double> std_v = {2,4,6,8,10};
            datalogger_client.log("std_v",std_v);

            datalogger_client.log("value_double",5);

            datalogger_client.log("value_string","Hello world!");
        }

        RCLCPP_INFO_STREAM_ONCE(node->get_logger(), "Logging ended. Close the `sas_datalogger_node` and check the resulting .mat file.");

        while(not kill_this_process)
        {
            std::this_thread::sleep_for(std::chrono::microseconds(100));
            rclcpp::spin_some(node);
        }
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR_STREAM_ONCE(node->get_logger(), std::string("::Exception::") + e.what());
    }

    sas::display_signal_handler_none_bug_info(node);
    return 0;
}
