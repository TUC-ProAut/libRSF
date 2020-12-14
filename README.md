# libRSF - A Robust Sensor Fusion Library
![GNSS Trajectory](./docs/img/AnimatedTrajectory2.gif)

The libRSF is an open source C++ library that provides the basic components for robust sensor fusion. It can be used to describe the estimation problem as a factor graph and solves it with least squares powered by the [Ceres Solver](http://ceres-solver.org//).
More information can be found under [libRSF - A Robust Sensor Fusion Library](https://www.tu-chemnitz.de/etit/proaut/libRSF).

Main features are:
- A sliding window filter for online applications, including marginalization.
- A set of predefined cost functions for various localization problems.
- Several robust error models for non-Gaussian problems, including self-tuning Gaussian mixtures.

## Installation

The libRSF is a CMake project that requires the installation of several dependencies.
For convenience, we provide a simple bash script that installs required packages.
It is tested **only for Ubuntu 18.04/20.04**:

```bash
  git clone https://github.com/TUC-ProAut/libRSF.git
  cd libRSF
  bash InstallDependecies.bash
```

Alternatively, you can install them by your own:

- **CMake** (>= 3.5)

  ```bash
  sudo apt-get install cmake
  ```

- **Eigen** (>= 3.3.5)

  **Only for Ubuntu 18.04**, you have to add a PPA for a more recent version of Eigen.

  ```
  sudo add-apt-repository ppa:nschloe/eigen-nightly
  ```

  For all versions of Ubuntu, you have to install Eigen.

  ```
  sudo apt-get update
  sudo apt-get install libeigen3-dev
  ```

- **Ceres** (>= 2.0) and its dependencies

  ```bash
  sudo apt-get install libgoogle-glog-dev
  sudo apt-get install libatlas-base-dev
  sudo apt-get install libsuitesparse-dev
    
  mkdir -p externals
  cd externals
  git clone https://ceres-solver.googlesource.com/ceres-solver
  cd ceres-solver
  mkdir build && cd build
  cmake -DEXPORT_BUILD_DIR=ON ..
  make all -j$(getconf _NPROCESSORS_ONLN)
  cd ../..
  ```

- **yaml-cpp**

  ```bash
  sudo apt-get install libyaml-cpp-dev
  ```

- **GeographicLib**

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

2. [How to use the robust Gaussian mixture models from our RA-L 2020 Paper?](docs/ROBUST.md)

3. [How to build your own application on top of the libRSF? (under construction)](docs/CUSTOM.md)

## Additional Information

### Citation

If you use this library for academic work, please cite it using the following BibTeX reference:

```latex
  @Misc{libRSF,
   author       = {Tim Pfeifer and Others},
   title        = {libRSF},
   howpublished = {\url{https://github.com/TUC-ProAut/libRSF}}
  }
```

This library also contains the implementation of [1-3]. Further references will be added with additional content.

[1] *Tim Pfeifer and Peter Protzel*, Expectation-Maximization for Adaptive Mixture Models in Graph Optimization, Proc. of Intl. Conf. on Robotics and Automation (ICRA), 2019, DOI: [10.1109/ICRA.2019.8793601](https://doi.org/10.1109/ICRA.2019.8793601)

[2] *Tim Pfeifer and Peter Protzel*, Incrementally learned Mixture Models for GNSS Localization, Proc. of Intelligent Vehicles Symposium (IV), 2019, DOI: [10.1109/IVS.2019.8813847](https://doi.org/10.1109/IVS.2019.8813847)

[3] *Tim Pfeifer and Sven Lange and Peter Protzel*, Advancing Mixture Models for Least Squares Optimization, Robotics and Automation Letters (RA-L), 2020 (coming soon)

### License

This work is released under the GNU General Public License version 3.
