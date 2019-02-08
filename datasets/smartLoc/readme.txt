The smartLoc Dataset

Copyright 2016-2018 by Pierre Reisdorf, Tim Pfeifer, Julia Breßler, Sven Bauer, Peter Weissig, Sven Lange (TU Chemnitz)

### The whole Dataset is licensed under a Creative Commons Attribution Share Alike 4.0 International license. ###

Every line of the included text file contains one measurement.
The string at the beginning identifies the measurement's type:

range3  - pseudorange measurements (atmospheric error and satellite clock bias already removed)
odom3   - odometry measurement
gt3     - ground truth

Every column holds an measurement element corresponding to its identifier:

## pseudoranges ##
1 - range3
2 - time stamp [s]
3 - pseudorange mean [m]
3 - pseudorange standard deviation [m]
4 - satellite position ECEF-X [m]
5 - satellite position ECEF-Y [m]
6 - satellite position ECEF-Z [m]
8 - satellite ID
9 - satellite elevation angle [deg]
10 - carrier-to-noise density ratio [dBHz]

## odometry ##
1 - odom3
2 - time stamp [s]
3 - velocity X-Axis [m/s]
4 - velocity Y-Axis [m/s]
5 - velocity Z-Axis [m/s]
6 - turn rate X-Axis [rad/s]
7 - turn rate Y-Axis [rad/s]
8 - turn rate Z-Axis [rad/s]
9 - standard deviation velocity X-Axis [m/s]
10 - standard deviation velocity Y-Axis [m/s]
11 - standard deviation velocity Z-Axis [m/s]
12 - standard deviation turn rate X-Axis [rad/s]
13 - standard deviation turn rate Y-Axis [rad/s]
14 - standard deviation turn rate Z-Axis [rad/s]

## ground truth ##
1 - gt3
2 - time stamp [s]
3 - ego position ECEF-X [m]
4 - ego position ECEF-Y [m]
5 - ego position ECEF-Z [m]

