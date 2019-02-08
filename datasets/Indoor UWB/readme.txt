The Labyrinth Dataset

Copyright 2017-2018 by Tim Pfeifer (TU Chemnitz)

### The whole Dataset is licensed under a Creative Commons Attribution Share Alike 4.0 International license. ###

Every line of the included text file contains one measurement.
The string at the beginning identifies the measurement's type:

range2      - range measurements
odom2diff   - differential drive odometry measurement
gt2         - ground truth

Every column holds an measurement element corresponding to its identifier:

## ranges ##
1 - range2
2 - time stamp [s]
3 - pseudorange mean [m]
3 - pseudorange standard deviation [m]
4 - UWB module position X [m]
5 - UWB module position Y [m]
6 - UWB module ID

## odometry ##
1 - odom2diff
2 - time stamp [s]
3 - velocity right wheel [m/s]
4 - velocity left wheel [m/s]
5 - velocity Y-Axis [m/s]
6 - distance between wheels [m]
9 - standard deviation velocity right wheel [m/s]
10 - standard deviation velocity left wheel [m/s]
11 - standard deviation velocity Y-Axis [m/s]

## ground truth ##
1 - gt2
2 - time stamp [s]
3 - ego position X [m]
4 - ego position Y [m]

