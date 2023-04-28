The Labyrinth Dataset

Copyright 2017-2023 by Tim Pfeifer (TU Chemnitz)

### The whole Dataset is licensed under a Creative Commons Attribution Share Alike 4.0 International license. ###

Every line of the included text file contains one measurement. Files ending with "Input" contain sensor data, files ending with "GT" contain the ground truth.
The string at the beginning identifies the measurement's type:

range2      - range measurements
odom2diff   - differential drive odometry measurement
point2      - ground truth

Every column holds an measurement element corresponding to its identifier:

## ranges ##
1 - "range2" [string]
2 - time stamp [s]
3 - range mean [m]
4 - range covariance [m]^2
5 - UWB module position X [m]
6 - UWB module position Y [m]
7 - UWB module ID
8 - UWB SNR (always set to 0)

## odometry ##
1 - "odom2diff" [string]
2 - time stamp [s]
3 - velocity right wheel [m/s]
4 - velocity left wheel [m/s]
5 - velocity Y-Axis [m/s]
6 - distance between wheels [m]
9 - covariance velocity right wheel [m/s]^2
10 - covariance velocity left wheel [m/s]^2
11 - covariance velocity Y-Axis [m/s]^2

## ground truth ##
1 - "point2" [string]
2 - time stamp [s]
3 - ego position X [m]
4 - ego position Y [m]
5-8 - covariance matrix 2x2 in row-major order, filled with zeros
