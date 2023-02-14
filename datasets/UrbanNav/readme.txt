The UrbanNav Dataset - Postprocessed with the RTKLIB using the gnss_converter (https://github.com/TUC-ProAut/gnss_converter)

Copyright 2021 by Li-Ta Hsu, Nobuaki Kubo, Weisong Wen, Wu Chen, Zhizhao Liu, Taro Suzuki, Junichi Meguro
Copyright 2023 by Tim Pfeifer (formatting and processing)

The original dataset is available under: https://github.com/IPNL-POLYU/UrbanNavDataset

Please cite:
Hsu, Li-Ta, Kubo, Nobuaki, Wen, Weisong, Chen, Wu, Liu, Zhizhao, Suzuki, Taro, Meguro, Junichi
UrbanNav:An Open-Sourced Multisensory Dataset for Benchmarking Positioning Algorithms Designed for Urban Areas
Proceedings of the 34th International Technical Meeting of the Satellite Division of The Institute of Navigation (ION GNSS+ 2021)
St. Louis, Missouri, September 2021, pp. 226-256.
https://doi.org/10.33012/2021.17895

### The whole Dataset is licensed under a Creative Commons Attribution NonCommercial Share Alike 4.0 International license. ###

Every line of the included text files contains one measurement. Files ending with "Input" contain sensor data, files ending with "GT" contain the ground truth.
The string at the beginning identifies the measurement's type:

pseudorange3  - pseudorange measurements (atmospheric error and satellite clock bias already removed)
imu           - inertial measurement
point3        - ground truth

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
9 - satellite System (1:GPS 2:SBAS 4:GLONASS 8:Galileo 16:QZSS 32:BeiDou)
10 - satellite elevation angle [deg]
11 - carrier-to-noise density ratio [dBHz]

## inertial measurement unit ##
1 - "imu" [string]
2 - time stamp [s]
3 - acceleration X-Axis [m^2/s]
4 - acceleration Y-Axis [m^2/s]
5 - acceleration Z-Axis [m^2/s]
6 - turn rate X-Axis [rad/s]
7 - turn rate Y-Axis [rad/s]
8 - turn rate Z-Axis [rad/s]
9 - covariance acceleration X-Axis [m^2/s]^2
10 - covariance acceleration Y-Axis [m^2/s]^2
11 - covariance acceleration Z-Axis [m^2/s]^2
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

