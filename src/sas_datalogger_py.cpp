/*
# Copyright (c) 2016-2023 Murilo Marques Marinho
#
#    This file is part of sas_datalogger.
#
#    sas_core is free software: you can redistribute it and/or modify
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
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/eigen.h>

#include <sas_datalogger/sas_datalogger_client.hpp>

namespace py = pybind11;
using DC = sas::DataloggerClient;

PYBIND11_MODULE(_sas_datalogger, m) {

    py::class_<DC> dc(m, "DataloggerClient");

    dc.def(py::init<const std::shared_ptr<rclcpp::Node>&,const size_t&>());
    dc.def("is_enabled",&DC::is_enabled,"Returns true if the client is connected to the datalogger server, false otherwise.");

    dc.def("log",py::overload_cast<const std::string&, const MatrixXd&>(&DC::log),"Logs a matrix.");
    dc.def("log",py::overload_cast<const std::string&, const VectorXd&>(&DC::log),"Logs a vector.");
    dc.def("log",py::overload_cast<const std::string&, const std::vector<double>&>(&DC::log),"Logs a vector.");
    dc.def("log",py::overload_cast<const std::string&, const double&>(&DC::log),"Logs a scalar.");
    dc.def("log",py::overload_cast<const std::string&, const std::string&>(&DC::log),"Logs a string.");
}
