#!/usr/bin/env bash

# libRSF - A Robust Sensor Fusion Library
#
# Copyright (C) 2019 Chair of Automation Technology / TU Chemnitz
# For more information see https://www.tu-chemnitz.de/etit/proaut/self-tuning
#
# libRSF is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# libRSF is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with libRSF.  If not, see <http://www.gnu.org/licenses/>.
#
# Author: Tim Pfeifer (tim.pfeifer@etit.tu-chemnitz.de)

# This script installs all dependencies of the libRSF

# stop script on any error
set -e

# config paths
readonly ceres_directory='ceres-solver'
readonly ortools_directory='or-tools'

# get linux version
readonly linux_distributor=$(lsb_release -is)
readonly linux_version=$(lsb_release -rs)

# check version
if [ "$linux_distributor" != "Ubuntu" ]
then
    echo "ERROR: Unsupported operation system, this script covers only Ubuntu systems!"
    exit 1 # terminate and indicate error
elif [ "$linux_version" == "16.04" ] # Ubuntu 16.04 does not support C++17 out-of-the-box
then
    echo "WARNING: C++17 is required! Please set it up manually!" 
fi

# install ceres dependencies
if [ "$linux_version" == "16.04" ] || [ "$linux_version" == "18.04" ] # Eigen < 3.3.5 is to old for libRSF & Ceres
then
  sudo add-apt-repository --yes ppa:kumarrobotics/backports
  sudo apt-get update
fi
sudo apt-get install libeigen3-dev --assume-yes

sudo apt-get install cmake --assume-yes

sudo apt-get install libgoogle-glog-dev --assume-yes
sudo apt-get install libatlas-base-dev --assume-yes
sudo apt-get install libsuitesparse-dev --assume-yes

# install other libRSF dependencies
sudo apt-get install libgeographic-dev --assume-yes
sudo apt-get install libyaml-cpp-dev --assume-yes

# install or-tools dependencies
sudo apt-get install build-essential zlib1g-dev --assume-yes
sudo apt-get install wget --assume-yes
sudo apt-get install tar --assume-yes

# prepare external dependencies
mkdir -p externals
cd externals

# install latest ceres version
if [ -d "$ceres_directory" ]
then
    cd "$ceres_directory"
    git fetch --all
    git reset --hard origin/master
    git pull origin master
else
    git clone https://ceres-solver.googlesource.com/ceres-solver "$ceres_directory"
    cd "$ceres_directory"
fi
mkdir -p build && cd build
cmake -DEXPORT_BUILD_DIR=ON ..
make all -j$(getconf _NPROCESSORS_ONLN)
cd ../..

# get or-tools binaries
if [ "$linux_version" == "20.04" ]
then
    if [ -d "$ortools_directory" ]
    then
        rm -rf "$ortools_directory"
    fi
    wget https://github.com/google/or-tools/releases/download/v7.7/or-tools_ubuntu-20.04_v7.7.7810.tar.gz
    tar -xzf or-tools_ubuntu-20.04_v7.7.7810.tar.gz
    mv -T or-tools_Ubuntu-20.04-64bit_v7.7.7810 "$ortools_directory"
    rm -rf or-tools_ubuntu-20.04_v7.7.7810.tar.gz
elif [ "$linux_version" == "18.04" ]
then
    if [ -d "$ortools_directory" ]
    then
        rm -rf "$ortools_directory"
    fi
    wget https://github.com/google/or-tools/releases/download/v7.3/or-tools_ubuntu-18.04_v7.3.7083.tar.gz
    tar -xzf or-tools_ubuntu-18.04_v7.3.7083.tar.gz
    mv -T or-tools_Ubuntu-18.04-64bit_v7.3.7083 "$ortools_directory"
    rm -rf or-tools_ubuntu-18.04_v7.3.7083.tar.gz
elif [ "$linux_version" == "16.04" ]
then
    if [ -d "$ortools_directory" ]
    then
        rm -rf "$ortools_directory"
    fi
    wget https://github.com/google/or-tools/releases/download/v7.3/or-tools_ubuntu-16.04_v7.3.7083.tar.gz
    tar -xzf or-tools_ubuntu-16.04_v7.3.7083.tar.gz
    mv -T or-tools_Ubuntu-16.04-64bit_v7.3.7083 "$ortools_directory"
    rm -rf or-tools_ubuntu-16.04_v7.3.7083.tar.gz
else
    echo "ERROR: Unsupported operation system (only Ubuntu 20.04, 18.04 and 16.04). Please download or-tools manually ..."
    cd ..
    exit 1 # terminate and indicate error
fi

# leave externals
cd ..
