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
readonly eigen_directory='eigen'

# config versions
readonly ceres_version='2.0.0'
readonly eigen_version='3.3.9'

# get linux version
readonly linux_distributor=$(lsb_release -is)
readonly linux_version=$(lsb_release -rs)

# check linux version
if [ "$linux_distributor" != "Ubuntu" ]
then
    echo "ERROR: Unsupported operation system, this script covers only Ubuntu systems!"
    exit 1 # terminate and indicate error
elif [ "$linux_version" == "16.04" ] # Ubuntu 16.04 does not support C++17 out-of-the-box
then
    echo "WARNING: C++17 is required! Please set it up manually!" 
fi

# install libRSF dependencies
sudo apt-get install cmake --assume-yes
sudo apt-get install libgeographic-dev --assume-yes
sudo apt-get install libyaml-cpp-dev --assume-yes

# prepare external dependencies
mkdir -p externals/install

# install eigen
if [ "$linux_version" == "16.04" ] || [ "$linux_version" == "18.04" ] # Eigen < 3.3.5 is to old for libRSF & Ceres
then
  echo "WARNING: Your Eigen version is below 3.3.5, we install it locally!"
  cd externals
  if [ -d "$eigen_directory" ]
  then
    cd "$eigen_directory"
    git checkout tags/"$eigen_version"
  else
    git clone --depth=1 --branch "$eigen_version" https://gitlab.com/libeigen/eigen.git "$eigen_directory"
    cd "$eigen_directory"
  fi
  
  mkdir -p build && cd build
  cmake -DCMAKE_INSTALL_PREFIX=../../install/ ..
  make install
  cd ../../..
else
    sudo apt-get install libeigen3-dev --assume-yes
fi

# install ceres dependencies
sudo apt-get install libgoogle-glog-dev --assume-yes
sudo apt-get install libatlas-base-dev --assume-yes
sudo apt-get install libsuitesparse-dev --assume-yes

# install ceres 
cd externals
if [ -d "$ceres_directory" ]
then
    cd "$ceres_directory"
    git checkout tags/"$ceres_version"
else
    git clone --depth=1 --branch "$ceres_version" https://ceres-solver.googlesource.com/ceres-solver "$ceres_directory"
    cd "$ceres_directory"
fi
mkdir -p build && cd build
if [ "$linux_version" == "16.04" ] || [ "$linux_version" == "18.04" ]
then
    cmake -DCMAKE_INSTALL_PREFIX=../../install/ -DEigen3_DIR=../install/share/eigen3/cmake ..
else
    cmake -DCMAKE_INSTALL_PREFIX=../../install/ ..
fi
make all -j$(getconf _NPROCESSORS_ONLN)
make install
cd ../..

# leave externals
cd ..
