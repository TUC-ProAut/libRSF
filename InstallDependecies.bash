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
  echo "WARNING: Your Eigen version is below 3.3.5, we have to activate a PPA! Please decide carefully if you want that!" 
  sudo add-apt-repository ppa:nschloe/eigen-nightly
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

# leave externals
cd ..
