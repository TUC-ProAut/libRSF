# libRSF - A Robust Sensor Fusion Library

The libRSF is an open source C++ library that provides several components that are required for robust sensor fusion. It can be used to describe the estimation problem as factor graph and solve it using the [Ceres Solver](http://ceres-solver.org//).
More information can be found under [libRSF - A Robust Sensor Fusion Library](https://www.tu-chemnitz.de/etit/proaut/libRSF).

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
      
## Applications
After building the library, some applications are provided. Usually they corresponding directly to a publication.

#### ICRA 2019
These two applications are made for the ICRA 2019 conferece, the corresponding paper is [1].
One can beused for GNSS datasets and calculated a 3D position in the ECEF frame, while the other one is for the 2D UWB ranging dataset.
To run them, the following syntax have to be used:

      libRSF/build/applications/ICRA19_GNSS     <input file> <output file> error: <error model>
      libRSF/build/applications/ICRA19_Ranging  <input file> <output file> error: <error model>
      
- **\<input file\>** is the dataset you want to process, the format is explained by readmes in the datasets folder
- **\<output file\>** is the estimated Trajectory. The output file contains several columns that represent timestamps and estimated positions:
      
      For 3D estimation:
      Column 1    - Timestamp [s]
      Column 2    - X coordinate in the ECEF frame [m]
      Column 3    - Y coordinate in the ECEF frame [m]
      Column 4    - Z coordinate in the ECEF frame [m]
      Column 5-13 - Covariance matrix of the estinmated position in row-major format (Currently not used!)
      
      For 2D estimation:
      Column 1    - Timestamp [s]
      Column 2    - X coordinate in a local frame [m]
      Column 3    - Y coordinate in a local frame [m]
      Column 4-7  - Covariance matrix of the estinmated position in row-major format (Currently not used!)
      
- **\<error model\>** is one of the following error models:

      gauss -     A Gaussian distribution
      dcs   -     Dynamic Covaraince Scaling
      cdce  -     Closed form Dynamic Covaraince Estimation
      mm    -     Max-Mixture (an approximation of a Gaussian mixture)
      sm    -     Sum-Mixture (an exact Gaussian mixture)
      stmm  -     Self-tuning Max-Mixture
      stsm  -     Self-tuning Sum-Mixture      

A full example could be:

      libRSF/build/applications/ICRA19_GNSS libRSF/datasets/smartLoc/Data_Berlin_Potsdamer_Platz_Web.txt Result_Berlin_Potsdamer_Platz_Web.txt error: gauss
