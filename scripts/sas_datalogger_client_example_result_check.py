#!/bin/python3

# Copyright (c) 2012-2026 Murilo Marques Marinho
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
# ################################################################

import glob
import os

import scipy.io


def main():
    current_working_directory = os.getcwd()
    filenames = glob.glob(rf"{current_working_directory}/*")
    print(filenames)
    filenames = glob.glob(rf"{current_working_directory}/*.mat")
    absolute_file_path = filenames[0]
    print(f"Try to open file {absolute_file_path}")
    mat = scipy.io.loadmat(absolute_file_path)

    if mat is not None:
        for key in mat:
            print(f"For element = {key}, the stored value was {mat[key]}")
            try:
                print(f"With shape = {mat[key].shape}")
            except Exception:
                pass
    else:
        print("Error loading mat file, or mat file empty")

if __name__ == "__main__":
    main()
