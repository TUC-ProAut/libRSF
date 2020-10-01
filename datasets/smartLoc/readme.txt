The smartLoc Dataset

Copyright 2016-2020 by Pierre Reisdorf, Tim Pfeifer, Julia Breßler, Sven Bauer, Peter Weissig, Sven Lange (TU Chemnitz)

### The whole Dataset is licensed under a Creative Commons Attribution Share Alike 4.0 International license. ###

Every line of the included text files contains one measurement. Files ending with "Input" contain sensor data, files ending with "GT" contain the ground truth.
The string at the beginning identifies the measurement's type:

range3  - pseudorange measurements (atmospheric error and satellite clock bias already removed)
odom3   - odometry measurement
point3  - ground truth

Every column holds an measurement element corresponding to its identifier:

## pseudoranges ##
1 - "pseudorange3" [string]
2 - time stamp [s]
3 - pseudorange mean [m]
4 - pseudorange covariance [m]^2
5 - satellite position ECEF-X [m]
6 - satellite position ECEF-Y [m]
7 - satellite position ECEF-Z [m]
8 - satellite ID
9 - satellite elevation angle [deg]
10 - carrier-to-noise density ratio [dBHz]

## odometry ##
1 - "odom3" [string]
2 - time stamp [s]
3 - velocity X-Axis [m/s]
4 - velocity Y-Axis [m/s]
5 - velocity Z-Axis [m/s]
6 - turn rate X-Axis [rad/s]
7 - turn rate Y-Axis [rad/s]
8 - turn rate Z-Axis [rad/s]
9 - covariance velocity X-Axis [m/s]^2
10 - covariance velocity Y-Axis [m/s]^2
11 - covariance velocity Z-Axis [m/s]^2
12 - covariance turn rate X-Axis [rad/s]^2
13 - covariance turn rate Y-Axis [rad/s]^2
14 - covariance turn rate Z-Axis [rad/s]^2

## ground truth ##
1 - "point3" [string]
2 - time stamp [s]
3 - ego position ECEF-X [m]
4 - ego position ECEF-Y [m]
5 - ego position ECEF-Z [m]
6-14 - covariance matrix 3x3 in row-major order, filled with zeros

