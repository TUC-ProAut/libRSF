The TUC SLAM Dataset

Copyright 2019-2023 by Peer Neubert, Stefan Schubert (TU Chemnitz)

Please Cite:
Neubert, P., Schubert, S. & Protzel, P. (2019) 
A neurologically inspired sequence processing model for mobile robot place recognition. 
In IEEE Robotics and Automation Letters (RA-L) and presentation at Intl. Conf. on Intelligent Robots and Systems (IROS). 
DOI: 10.1109/LRA.2019.2927096 

### The whole Dataset is licensed under a Creative Commons Attribution NonCommercial Share Alike 4.0 International license. ###

Every line of the included text files contains one measurement. Files ending with "Input" contain sensor data, files ending with "GT" contain the ground truth.
The string at the beginning identifies the measurement's type:

loop        - loop closure measurements
odom2       - 2D odometry measurement
point2      - 2D ground truth

Every column holds an measurement element corresponding to its identifier:

## loop closures ##
1 - "loop" [string]
2 - time stamp (first connected pose) [s]
3 - time stamp loop (second connected pose) [s]
4 - similarity score between 0 and 1 (one means highest similarity)

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

