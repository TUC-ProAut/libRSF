The M3500/W3500 Ranging Dataset

Copyright 2022-2023 by Tim Pfeifer (TU Chemnitz)

M3500 is based on the M3500 SLAM dataset from Luca Carlone.
Reference:
L. Carlone and A. Censi. 
From Angular Manifolds to the Integer Lattice: Guaranteed Orientation Estimation With Application to Pose Graph Optimization. 
IEEE Trans. Robotics, 30(2):475-492, 2014.

W3500 is based on W10000 SLAM dataset from GTSAM.
https://github.com/borglab/gtsam/tree/develop/examples/Data

### The whole Dataset is licensed under a Creative Commons Attribution Share Alike 4.0 International license. ###

Every line of the included text file contains one measurement. Files ending with "Input" contain sensor data, files ending with "GT" contain the ground truth.
The string at the beginning identifies the measurement's type:

range2      - range measurements
odom2       - 2D odometry measurement
point2      - 2D ground truth

Every column holds an measurement element corresponding to its identifier:

## ranges ##
1 - "range2" [string]
2 - time stamp [s]
3 - pseudorange mean [m]
4 - pseudorange covariance [m]^2
5 - UWB module position X [m]
6 - UWB module position Y [m]
7 - UWB module ID

## odometry ##
1 - "odom2" [string]
2 - time stamp [s]
3 - velocity X-Axis [m/s]
4 - velocity Y-Axis [m/s]
5 - turn rate [rad/s]
6 - covariance velocity X-Axis [m/s]^2
7 - covariance velocity Y-Axis [m/s]^2
8 - covariance turn rate [rad/s]^2

## ground truth ##
1 - "point2" [string]
2 - time stamp [s]
3 - ego position X [m]
4 - ego position Y [m]
5-8 - covariance matrix 2x2 in row-major order, filled with zeros
