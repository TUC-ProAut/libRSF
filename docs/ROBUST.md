## RA-L 2020 Applications

In 2020 we introduced a new representation of Gaussian mixtures for factor graphs that reliy on a least squares back-end.
Previous models are either approximations like Max-Mixture [2] or ill-posed like Sum-Mixture which is based on [3].
We proposed the Max-Sum-Mixture model [1] which combines the advantages of both approaches without their drawbacks.
Our model is an almost linear and exact representation of a GMM for least squares.

Within the libRSF, we provide the implementations of all three models: Max-Mixture, Sum-Mixture and the proposed Max-Sum-Mixture.
Beside the raw error models, the simple example application from [1] is also included.

### Robust Gaussian Mixtures
![GNSS Trajectory](./img/Error2D.png)

The included one- or two-dimensional example demonstrate the convergence speed and accuracy of the three available models. A single cost function describes a Gaussian mixture as shown in the figure above (2D case). Starting from a set of linearly spaced initial points, several optimizations are performed individually to capture the dependence between initialization and convergence. 

#### Using Matlab Scripts

For convenience, we provide a number of Matlab scripts, that generate the data, write it to text files, call the compiled binary and parse the generated output. You simply have to start Matlab in the corresponding folder:

```bash
cd libRSF/matlab
matlab
```

Then call the main script to start the full Monte Carlo Evaluation:

```matlab
run Estimation/Robust_Models/MonteCarloComparison.m
```

Alternatively, you can run the evaluation of a single GMM separately with:

```matlab
run Estimation/Robust_Models/CompareRobustModels1D.m
% or
run Estimation/Robust_Models/CompareRobustModels2D.m
```

All scripts print their results to console and generate a `Results` folder that contains different plots for visualization, like the one shown on top of this page.
Feel free to try them out and to play with the configuration parameter at the beginning of each script.

#### Using the Binaries

If you do not want to use Matlab, you can also execute the compiled binaries directly. 
Since this examples use just a small subset of functionalities from the libRSF, it is important to mention that some input/output/config parameters are not used. To run both examples, the same following syntax have to be applied:

      libRSF/build/applications/App_Robust_Models_1D  empty empty  <output file> <number points> <range points> <error model> <model parameters>
      libRSF/build/applications/App_Robust_Models_2D  empty empty  <output file> <number points> <range points> <error model> <model parameters>

- the parameters start with two placeholder strings **"empty"** that are separated by a space. These are required for compatibility reasons.

- **\<output file\>** is the filename of a text file that will contain the output of the example application. Each line starts with an identifying string, followed by a number of floating point numbers.
  The lines can be grouped into four blocks:
  1. Point1D/Point2D &ndash; The initial points before the optimization.
  2. Point1D/Point2D &ndash; The same points after the optimization.
  3. Solver Summary &ndash; The duration and number of iterations of the optimization.
  4. Cost Surface Information &ndash; Cost, gradient and Hessian for a specific point.
  
  Each line corresponds to the one from the other block. So line 1 of block 2 is the point that results from an optimization with the initial value that is stored in line 1 of block 1. The meaning of a line in a block is defined by its identifying string.
  
    For the 1D case:
  
    ```
  ## Point 1D ##
  1   - "point1"
  2   - this is 0 for initial values, 1 for optimized values
  3   - X coordinate
  4   - not used
  
  ## Cost surface information ##
  1   - "cost_gradient"
  2   - X coordinate
  3   - cost value the specific coordinate
  4   - gradient of the cost w.r.t. the optimized variable
  5   - Hessian w.r.t. the optimized variable
    ```
  
    For the 2D case:
  
    ```
  ## Point 2D ##
  1   - "point2"
  2   - this is 0 for initial values, 1 for optimized values
  3   - X coordinate
  4   - Y coordinate
  5-8 - not used
  
  ## Cost surface information ##
  1   - "cost_gradient"
  2   - X coordinate
  3   - Y coordinate
  4   - cost value the specific coordinate
  4-5 - gradient of the cost w.r.t. the optimized variable (as vector)
  6-9 - Hessian w.r.t. the optimized variable (as row-major matrix)
    ```
  
    For both cases:
  
    ```
  ## Timing information ##
  1   - "solver_summary"
  2   - not used
  3   - not used
  4   - solver time in seconds
  5   - not used
  6   - not used
  7   - solver iterations
  8   - not used
    ```
  
- **\<number points\>** is the number of points per dimension that are generated as linearly spaced grid. In the 2D case, 10 means a 10-by-10 grid.

- **\<range points\>** is the range of this linearly spaced grid. The grid is always centered around 0, so a value of 8 means it is distributed between -4 and 4.

- **\<error model\>** is a string that represents one of the following error models:

      Gaussian    -     A simple Gaussian model
      MaxMix      -     Max-Mixture (an approximation of a Gaussian mixture)
      SumMix      -     Sum-Mixture (an exact Gaussian mixture with a ill-posed jacobian)
      MaxSumMix   -     Max-Sum-Mixture (our proposed model)
      DCS         -     Dynamic Covariance Scaling (an M-estimator)

- **\<model parameters\>** are the parameter that describe the Gaussian mixture model. Each entry (separated by a space) describes one parameter of a model with two Gaussian components.

  For the 1D case:
  ```
  1   -   mean of the first component
  2   -   mean of the second component
  3   -   standard deviation of the first component
  4   -   standard deviation of the second component
  5   -   weight of the first component
  6   -   weight of the second component
  ```
  For the 2D case:
  ```
  1-2  -   mean vector of the first component
  3-4  -   mean vector of the second component
  3-8  -   standard deviation matrix of the first component (row-major)
  9-12 -   standard deviation matrix of the second component (row-major)
  13   -   weight of the first component
  14   -   weight of the second component
  ```


A full example could be:

      ./App_Robust_Models_2D empty empty Data_2D_Output.txt 10 8 MaxSumMix 0 0 1 2 0.5 0 0 1 2 0 0 5 0.35 0.65
                                                            ^  ^     ^     ^ ^ ^ ^  ^  ^ ^ ^ ^ ^ ^ ^  ^    ^
                                                            |  |     |     | | | |  |  | | | | | | |  |    |
                      Number of points (per dimension) -----+  |     |     | | | |  |  | | | | | | |  |    |
                                       Range of points --------+     |     | | | |  |  | | | | | | |  |    |
                                 Type of mixture model --------------+     \_/ | |  |  | | | | | | |  |    |
                                       Mean of comp. 1 ---------------------+  \_/  |  | | | | | | |  |    |
                                       Mean of comp. 2 -------------------------+   \______/ | | | |  |    |
                         Standard deviation of comp. 1 --------------------------------+     \_____/  |    |
                         Standard deviation of comp. 2 -----------------------------------------+     |    |
                                     Weight of comp. 1 -----------------------------------------------+    |
                                     Weight of comp. 2 ----------------------------------------------------+


### Point set Registration

The point set registration example from [1] is currently not available for the Ceres implementation. Please have a look at our GTSAM version.

### References

If you are interested in this work, feel free to have a look at our recent paper:

[1] *Tim Pfeifer and Sven Lange and Peter Protzel*, Advancing Mixture Models for Least Squares Optimization, Robotics and Automation Letters (RA-L), 2020

BibTeX:

```latex
    @Article{Pfeifer2020,
      author       = {Tim Pfeifer, Sven Lange and Peter Protzel},
      journal      = {Robotics and Automation Letters (RA-L)},
      title        = {Advancing Mixture Models for Least Squares Optimization},
      year         = {2020},
    }
```

#### The Work of Others

Our approach is strongly inspired by the former work of Olsen and Rosen, therefore we refer also to their publications:

[2] *Edwin Olson and Pratik Agarwal*, Inference on networks of mixtures for
robust robot mapping, Int. Journal of Robotics Research, 2013

[3] *David M. Rosen and Michael Kaess and John J. Leonard*, Robust Incremental Online Inference Over Sparse
Factor Graphs: Beyond the Gaussian Case, Proc. of Int. Conf. on Robotics and Automation (ICRA), 2013
