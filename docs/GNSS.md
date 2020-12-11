## Legacy Applications
These applications are mainly developed for early versions of the libRSF.
While they are still supported, their result might differ from the original work.
To reproduce the published result, you can use the 1.1.0 tag.

### ICRA 2019 Applications

These two applications are made for the ICRA 2019 conference, the corresponding paper is [1].
One can be used with GNSS data and estimates a 3D position in the ECEF frame, while the other one is for 2D ranging datasets.
To run them, the following syntax have to be used:

      libRSF/build/applications/ICRA19_GNSS     <input file> <output file> error: <error model>
      libRSF/build/applications/ICRA19_Ranging  <input file> <output file> error: <error model>

- **\<input file\>** is the dataset you want to process, the format is explained by readme files in the datasets folder.

- **\<output file\>** is the estimated Trajectory. The output file contains several columns that represent timestamps and estimated positions:

      For 3D estimation:
      Column 1    - "point3" [string]
      Column 2    - Timestamp [s]
      Column 3    - X coordinate in the ECEF frame [m]
      Column 4    - Y coordinate in the ECEF frame [m]
      Column 5    - Z coordinate in the ECEF frame [m]
      Column 6-14 - Covariance matrix of the estimated position in row-major format (Currently not used!)
    
      For 2D estimation:
      Column 1    - "point2" [string]
      Column 2    - Timestamp [s]
      Column 3    - X coordinate in a local frame [m]
      Column 4    - Y coordinate in a local frame [m]
      Column 5-8  - Covariance matrix of the estimated position in row-major format (Currently not used!)

- **\<error model\>** is one of the following error models:

      gauss -     A Gaussian distribution
      dcs   -     Dynamic Covariance Scaling
      cdce  -     Closed form Dynamic Covariance Estimation
      mm    -     Max-Mixture (an approximation of a Gaussian mixture)
      sm    -     Sum-Mixture (an exact Gaussian mixture)
      stmm  -     Adaptive Max-Mixture using the EM Algorithm
      stsm  -     Adaptive Sum-Mixture using the EM Algorithm  

A full example could be:

      libRSF/build/applications/ICRA19_GNSS libRSF/datasets/smartLoc/Berlin_Potsdamer_Platz_Input.txt Result_Berlin_Potsdamer_Platz_Web.txt error: gauss

### IV 2019 Application

These application is made for the IV 2019 conference, the corresponding paper is [2].
It can be used with GNSS data and estimates a 3D position in the ECEF frame.
To run them, the following syntax have to be used:

      libRSF/build/applications/IV19_GNSS     <input file> <output file> error: <error model>

- **\<input file\>** is the dataset you want to process, the format is explained by readme files in the datasets folder.
- **\<output file\>** is the estimated Trajectory. The output file contains several columns that represent timestamps and estimated positions:

      Column 1    - "point3" [string]
      Column 2    - Timestamp [s]
      Column 3    - X coordinate in the ECEF frame [m]
      Column 4    - Y coordinate in the ECEF frame [m]
      Column 5    - Z coordinate in the ECEF frame [m]
      Column 6-14 - Covariance matrix of the estimated position in row-major format (Currently not used!)

- **\<error model\>** is one of the following error models:

      gauss     -     A Gaussian distribution
      dcs       -     Dynamic Covariance Scaling
      cdce      -     Closed form Dynamic Covariance Estimation
      mm        -     Max-Mixture (an approximation of a Gaussian mixture)
      sm        -     Sum-Mixture (an exact Gaussian mixture)
      stmm      -     Adaptive Max-Mixture using the EM Algorithm
      stsm      -     Adaptive Sum-Mixture using the EM Algorithm
      stmm_vbi  -     Incrementally learned Max-Mixture using the VBI Algorithm
      stsm_vbi  -     Incrementally learned Sum-Mixture using the VIB Algorithm  

A full example could be:

      libRSF/build/applications/IV19_GNSS libRSF/datasets/smartLoc/Berlin_Potsdamer_Platz_Input.txt Result_Berlin_Potsdamer_Platz_Web.txt error: gauss

### References

[1] *Tim Pfeifer and Peter Protzel*, Expectation-Maximization for Adaptive Mixture Models in Graph Optimization, Proc. of Intl. Conf. on Robotics and Automation (ICRA), 2019, DOI: [10.1109/ICRA.2019.8793601](https://doi.org/10.1109/ICRA.2019.8793601)

[2] *Tim Pfeifer and Peter Protzel*, Incrementally learned Mixture Models for GNSS Localization, Proc. of Intelligent Vehicles Symposium (IV), 2019, DOI: [10.1109/IVS.2019.8813847](https://doi.org/10.1109/IVS.2019.8813847)

BibTeX:

```latex
    @InProceedings{Pfeifer2019,
    author    = {Tim Pfeifer and Peter Protzel},
    title     = {Expectation-Maximization for Adaptive Mixture Models in Graph Optimization},
    booktitle = {Proc. of Intl. Conf. on Robotics and Automation (ICRA)},
    doi       = {10.1109/ICRA.2019.8793601},
    year      = {2019}
    }

    @InProceedings{Pfeifer2019a,
    author    = {Tim Pfeifer and Peter Protzel},
    title     = {Incrementally learned Mixture Models for GNSS Localization},
    booktitle = {Proc. of Intelligent Vehicles Symposium (IV)},
    doi       = {10.1109/IVS.2019.8813847},
    year      = {2019}
    }
```
