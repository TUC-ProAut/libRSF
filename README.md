# libRSF - A Robust Sensor Fusion Library
![GNSS Trajectory](./docs/img/AnimatedTrajectory2.gif)

The libRSF is an open source C++ library that provides the basic components for robust sensor fusion. It can be used to describe an estimation problem as a factor graph and solves it with least squares, powered by the [Ceres Solver](http://ceres-solver.org//).
More information can be found under [libRSF - A Robust Sensor Fusion Library](https://www.tu-chemnitz.de/etit/proaut/libRSF).

Main features are:
- A sliding window filter for online applications, including marginalization.
- A set of predefined cost functions for various localization problems.
- Several robust error models for non-Gaussian problems, including self-tuning Gaussian mixtures.

## Build and Test Status

| Platform     | Status  |
|:------------:|:-------------:|
| Ubuntu 22.04 | ![Focal Build](https://github.com/TUC-ProAut/libRSF/workflows/Jammy%20CI/badge.svg) |
| Ubuntu 20.04 | ![Focal Build](https://github.com/TUC-ProAut/libRSF/workflows/Focal%20CI/badge.svg) |

## Installation

The libRSF is a CMake project that requires the installation of several dependencies.
For convenience, we provide a simple bash script that installs required packages.
It is tested **only for Ubuntu 18.04/20.04**:

```bash
  git clone https://github.com/TUC-ProAut/libRSF.git
  cd libRSF
  bash InstallDependencies.bash
```

Alternatively, you can install them by your own:

+ **CMake** (>= 3.5)

  ```bash
  sudo apt-get install cmake
  ```

+ **Eigen** (>= 3.3.5)

  **Only for Ubuntu 18.04**, you have to install a current version of Eigen locally.

  ```bash
  mkdir -p externals/install
  git submodule update --init externals/eigen
  
  cd externals/eigen
  mkdir -p build && cd build
  cmake -DCMAKE_INSTALL_PREFIX=../../install/ ..
  make install
  cd ../../..
  ```

  **For  Ubuntu >= 20.04** you can install Eigen straight-forward.

  ```bash
  sudo apt-get install libeigen3-dev
  ```

+ **Ceres** (>= 2.0) and its dependencies

  ```bash
  sudo apt-get install libgoogle-glog-dev
  sudo apt-get libgflags-dev
  sudo apt-get install libatlas-base-dev
  sudo apt-get install libsuitesparse-dev
  
  mkdir -p externals/install  
  git submodule update --init externals/ceres-solver
  
  cd externals/ceres-solver
  mkdir build && cd build
  cmake -DEigen3_DIR=../install/share/eigen3/cmake -DCMAKE_INSTALL_PREFIX=../../install/ ..
  make all -j$(getconf _NPROCESSORS_ONLN)
  make install
  cd ../..
  ```

+ **yaml-cpp**

  ```bash
  sudo apt-get install libyaml-cpp-dev
  ```

+ **GeographicLib**

  ```bash
  sudo apt-get install libgeographic-dev
  ```

The library and its applications can be build following this instructions:

```bash
  git clone https://github.com/TUC-ProAut/libRSF.git
  cd libRSF
  mkdir build && cd build
  cmake ..
  make all -j$(getconf _NPROCESSORS_ONLN)
```

You can install the libRSF using:

```bash
  make install
```

And remove it using:

```bash
  make uninstall
```

## Usage

After building the library, some applications are provided which correspond directly to a publication.
The following pages give you an overview, how to use them or how to build a custom application using the libRSF:

1. [How to use the robust GNSS localization from our ICRA 2019 or IV 2019 paper?](docs/GNSS.md)

2. [How to use the robust Gaussian mixture models from our RA-L 2021 Paper?](docs/ROBUST.md)

3. [How to build your own application on top of the libRSF?](docs/CUSTOM.md)

## Additional Information

### Citation

If you use this library for academic work, please cite it using the following BibTeX reference:

```tex
  @Misc{libRSF,
   author       = {Tim Pfeifer and Others},
   title        = {libRSF},
   howpublished = {\url{https://github.com/TUC-ProAut/libRSF}}
  }
```

This library also contains the implementation of [1-3]. Further references will be added with additional content.

[1] *Tim Pfeifer and Peter Protzel*, Expectation-Maximization for Adaptive Mixture Models in Graph Optimization, Proc. of Intl. Conf. on Robotics and Automation (ICRA), 2019, DOI: [10.1109/ICRA.2019.8793601](https://doi.org/10.1109/ICRA.2019.8793601)

[2] *Tim Pfeifer and Peter Protzel*, Incrementally learned Mixture Models for GNSS Localization, Proc. of Intelligent Vehicles Symposium (IV), 2019, DOI: [10.1109/IVS.2019.8813847](https://doi.org/10.1109/IVS.2019.8813847)

[3] *Tim Pfeifer and Sven Lange and Peter Protzel*, Advancing Mixture Models for Least Squares Optimization, Robotics and Automation Letters (RA-L), 2021, DOI: [10.1109/LRA.2021.3067307](https://dx.doi.org/10.1109/LRA.2021.3067307)

### License

This work is released under the GNU General Public License version 3.