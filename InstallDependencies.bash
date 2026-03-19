#!/usr/bin/env bash

# libRSF - A Robust Sensor Fusion Library
#
# Copyright (C) 2023 Chair of Automation Technology / TU Chemnitz
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

# get linux version
readonly linux_distributor=$(lsb_release -is)
readonly linux_version=$(lsb_release -rs)

# check linux version
if [ "$linux_distributor" != "Ubuntu" ]; then
    echo "ERROR: Unsupported operation system, this script covers only Ubuntu systems!"
    exit 1 # terminate and indicate error
elif [ "$linux_version" != "22.04" ] && [ "$linux_version" != "24.04" ]; then
    echo "WARNING: This script is tested only for Ubuntu 22.04 and 24.04!"
fi

# function to check dependencies
install_if_not_exist ()
{
  if dpkg -s $1 &>/dev/null; then
    PKG_EXIST=$(dpkg -s $1 | grep "install ok installed")
    if [ -z "$PKG_EXIST" ]; then
      sudo apt-get install $1 --assume-yes
    fi
  else
    sudo apt-get install $1 --assume-yes
  fi
}

# Bugfix for Ubuntu Jammy
if [ "$linux_version" == "22.04" ]; then
  install_if_not_exist libunwind-dev
fi

# install libRSF dependencies
install_if_not_exist build-essential
install_if_not_exist cmake
install_if_not_exist ninja-build
install_if_not_exist libyaml-cpp-dev
install_if_not_exist libeigen3-dev

# GeographicLib package name changed in Ubuntu 24.04
if [ "$linux_version" == "24.04" ]; then
  install_if_not_exist libgeographiclib-dev
else
  install_if_not_exist libgeographic-dev
fi

# install ceres dependencies
install_if_not_exist libgoogle-glog-dev
install_if_not_exist libgflags-dev
install_if_not_exist libatlas-base-dev
install_if_not_exist libsuitesparse-dev

# install ceres locally (not yet available as system package on all versions)
if compgen -G "externals/install/lib/libceres*" > /dev/null 2>&1; then
  echo "Ceres already installed, skipping build."
else
  mkdir -p externals/install
  git submodule update --init externals/ceres-solver
  cd externals/ceres-solver

  mkdir -p build && cd build
  cmake -G Ninja -DCMAKE_INSTALL_PREFIX=../../install/ -DBUILD_EXAMPLES=OFF -DBUILD_TESTING=OFF -DSCHUR_SPECIALIZATIONS=OFF -DSUITESPARSE=OFF -DCXSPARSE=OFF ..
  ninja
  ninja install
  cd ../..
fi

# leave externals
cd ..
