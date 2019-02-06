# libRSF - A Robust Sensor Fusion Library

The libRSF is an open source C++ library that provides several components that are required for robust sensor fusion. It can be used to describe the estimation problem as factor graph and solve it using the [Ceres Solver](http://ceres-solver.org//).
More information can be found under www.tu-chemnitz.de/etit/proaut/en/research/self-tuning.html.

Main features are:
- A set of predefined error functions for localization problems.
- Several robust error models for non-Gaussian problems.
- A sliding window filter for online applications.

### License

This work is released under the GNU General Public License version 3.

### Citation

## Installation

The libRSF is a CMake project that requires the installation of the following dependencies:

- CMake (>= 2.8)

      sudo apt-get install cmake
      
- Eigen (>= 3.3)

      sudo apt-get install libeigen3-dev
      
- Ceres (>= 2.0) and its dependencies

      sudo apt-get install libgoogle-glog-dev
      sudo apt-get install libatlas-base-dev
      sudo apt-get install libsuitesparse-dev
      
      git clone https://ceres-solver.googlesource.com/ceres-solver
      cd ceres-solver
      mkdir build && cd build
      cmake ..
      make all -j8
      make install
      
The library and its applications can be build following this instructions:

      git clone https://github.com/tipf/libRSF.git
      cd libRSF
      mkdir build && cd build
      cmake ..
      make all -j8
      
## Application
