#!/usr/bin/env bash

# libRSF - A Robust Sensor Fusion Library
#
# Copyright (C) 2023 Chair of Automation Technology / TU Chemnitz
# Copyright (C) 2026 Tim Pfeifer
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

# This script builds the libRSF using CMake presets

# stop script on any error
set -e

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
  echo "Usage: $0 [preset]"
  echo "Build libRSF using a CMake preset (default: release)."
  echo "Available presets: release, debug, test"
  exit 0
fi

preset="${1:-release}"

cmake --preset "${preset}"
cmake --build --preset "${preset}" -j"$(getconf _NPROCESSORS_ONLN)"
