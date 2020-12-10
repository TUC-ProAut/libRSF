## Error Models
With the libRSF, we want to provide a broad selection of methods to solve non-linear least squares problems robustly. Robustness against non-Gaussian outlier is the main challenge there and and the required models are as diverse as the possible applications. All oth them describe the log-likelihod of a assumed noise distributions. Therefore they cam be summarized as probabilistic models.

In the following, we provide an overview over the available types of models.

### Gaussian

The most simple and natural model for least squares is the Gaussian or normal distribution. Its log-likelihood is a squared function and therefore, it is quite sensitive to outliers.

Its implementation can be found here: [Gaussian.h](../include/error_models/Gaussian.h)

#### Gaussian Diagonal

The diagonal Gaussian is a a simplified variant with a diagonal covariance matrix. Often, this reduced version is sufficient to weight residuals without correlation and it is noticeably faster the the full version.

#### Gaussian Full

The full Gaussian model without any reduction.

### Gaussian Mixtures

As linear superposition (sum) of multiple Gaussians, the Gaussian mixture model (GMM) is one of the flexible models. It can represent various shapes like skewed or heavy-tailed distributions, but still its piece-wise close to Gaussian.
Over the years, several implementation of GMMs for least squares where proposed.

#### Max-Mixture (MM)

Max-Mixture, as proposed by [1] , is an approximation for Gaussian Mixtures. Instead of evaluating the full sum, it used just the (locally) most dominant component of the GMM.  This simplification works well if the components do not overlap to much and allows efficient optimization.
On the downside, it can lead to additional minima compared to the original GMM.

Its implementation can be found here: [MaxMixture.h](../include/error_models/MaxMixture.h)

#### Sum-Mixture (SM)

Sum-Mixture is based on the work from Rosen et al. [2] and represents the GMM exactly. Unfortunately, it has a strong nonlinearity and its challenging derivative makes it hard to optimize.
For one-dimensional models its applicable, but for anything beyond it does not work well.

Its implementation can be found here: [SumMixture.h](../include/error_models/SumMixture.h)

#### Max-Sum-Mixture (MSM)

Max-Sum-Mixture is a novel [3] Formulation based on Sum-Mixture, but with the good convergence properties of Max-Mixture. It is an exact GMM representation and our recommendation if you want tor represent a GMM in least squares.

Its implementation can be found here: [MaxSumMixture.h](../include/error_models/MaxSumMixture.h)

### Dynamic Models

Dynamic models introduce new state variables to the optimization problem. Therefore, they are often computational more expensive than static models.
Their advantage is a cost surface, which is easier to optimize, albeit is has more dimensions.

#### Switchable Constraints (SC)

Switchable constraint [4] introduced a so-called "switch variable" that lays in the interval [0;1] and scales the optimized residual. With an additional prior, the sensitivity of the down-weighting can be set.
SC is quite popular for SLAM applications, were extreme outliers can occur. However, it is less effective for more compact non-Gaussian distributions that  can be seen in GNSS localization.

Its implementation can be found here: [SwitchableConstraints.h](../include/error_models/SwitchableConstraints.h)

#### Dynamic Covariance Estimation (DCE)

Dynamic Covariance Estimation [5] is very similar to Switchable Constraints, but adapts the covariance matrix itself instead of additional wheigting. In consequence, the arbitrary tuning parameter for the weight prior can be omitted.
DCE performs better than SC for compact bounded distributions, while SE is better for extreme outliers.

Its implementation can be found here: [DynamicCovarianceEstimation.h](../include/error_models/DynamicCovarianceEstimation.h)

### M-Estimators (Loss functions)

M-Estimators are robust loss functions that decrease the quadratic influence for large residuals. A variety of them is defined in the Ceres back-end itself.
In difference to the full error models, they get usually combined with a simple Gaussian model.

A few additional loss functions for the libRSF can be found here: [LossFunction.h](../include/error_models/LossFunction.h)

#### Dynamic Covariance Scaling (DCS)

Dynamic Covariance Scaling [6] is the closed-form equivalent of Switchable Constraints. It is computationally more efficient but required a better initialization to ensure convergence. Its characteristics are almost the same.

#### closed-form Dynamic Covariance Estimation (cDCE)

While DCS [6] id the closed-form of SC[4], we provide also a closed-form version of DCE [5]. Similar to DCS is it computationally more efficient than the full DCE formulation.

### Technical Interface

All error models (not the loss functions!) are derived from the ErrorModel class from [ErrorModel.h](../include/error_models/ErrorModel.h).  They accept a vector as unweighted residual or error  and write the weighted error to a ceres-compatible pointer interface. They have to be used in conjunction with factors (residual functions) that define some deterministic error. For more information about factors see [here](CUSTOM_FACTORS.md).

```
                    ------------                     -------------
   States    ----->|            |----->       ----->|             |----->
    and      ----->|   Factor   |-----> Error ----->| Error Model |-----> Weighted Error
Measurements ----->|            |----->       ----->|			  |----->
                    ------------                     ------------- 	
```

Both, input and output of the error model can have a different dimensionality, which has to be given as template argument. Pleas note, that due to implementation reasons, the input can not have more dimensions as the output.
Additionally, all error models have a "Enable" flag to activate or deactivate them. Disabled, they simply pass-through the error and fill unused dimension with zero.

Below you can see an example for a 2D Gaussian: 

```c++
// oputput dimension ----------------------| 
// input dimension ---------------------|  |
// base class-------------------|       |  |
//                              V       V  V
  class Gaussian2D : public ErrorModel <2, 2>
  {
  public:

      Gaussian2D() = default;

      void setCovarianceMatrix(const Matrix22 &CovMat)
      {
        /** square root information is more efficient to apply */
        _SqrtInformation = InverseSquareRoot<Dim, double>(CovMat);
      }

      template <typename T>
      bool weight(const VectorT<T, 2> &RawError, T* WeightedError) const
      {
        /** wrap raw pointer to vector*/
        VectorRef<T, 2> ErrorMap(WeightedError);

        if(this->_Enable)
        {
          /** scale with full information matrix */
          ErrorMap = _SqrtInformation.template cast<T>() * RawError;
        }
        else
        {
          /** pass-trough if error model is disabled*/
          ErrorMap = RawError;
        }
        return true;
      }
     
  private:
      Matrix22 _SqrtInformation;
  };
```

### References

[1] *Edwin Olson and Pratik Agarwal*, Inference on networks of mixtures for robust robot mapping,
Int. Journal of Robotics Research, 2013

[2] *David M. Rosen and Michael Kaess and John J. Leonard*, Robust Incremental Online Inference Over Sparse Factor Graphs: Beyond the Gaussian Case,
Proc. of Int. Conf. on Robotics and Automation (ICRA), 2013

[3] *Tim Pfeifer and Sven Lange and Peter Protzel*, Advancing Mixture Models for Least Squares Optimization, 
Robotics and Automation Letters (RA-L), 2020 (coming soon)

[4] *Niko Sünderhauf and Peter Protzel*, Switchable constraints for robust pose graph SLAM, 
Proc. of Int. Conf. on Intelligent Robots and Systems (IROS), 2012

[5] *Tim Pfeifer and Sven Lange and Peter Protzel*, Dynamic Covariance Estimation -- A parameter free approach to robust Sensor Fusion 
Proc. of Int. Conf. on Multisensor Fusion and Integration for Intelligent Systems (MFI), 2017

[6] *Pratik Agarwal, Gian Diego Tipaldi, Luciano Spinello, Cyrill Stachniss, and Wolfram Burgard*, Robust Map Optimization using Dynamic Covariance Scaling Proc. of Int. Conf. on Robotics and Automation (ICRA), 2013