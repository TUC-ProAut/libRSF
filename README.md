# libRSF - A Robust Sensor Fusion Library

The libRSF is an open source C++ library that provides several components that are required for robust sensor fusion. It can be used to describe the estimation problem as factor graph and solve it using the [Ceres Solver](http://ceres-solver.org//).

Main features are:
- A set of predefined error functions for localization problems.
- Several robust error models for non-Gaussian problems.
- A sliding window filter for online applications.
