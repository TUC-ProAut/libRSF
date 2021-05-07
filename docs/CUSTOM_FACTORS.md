## Factors
Factors or residual functions are a set of deterministic error functions that define the optimization problem that we want so solve. The name "factor" comes from the factor graph approach, where each residual term corresponds to a factor in the maximum a posteriori product.

The libRSF provides a broad set of absolute and relative factors, representing different kinds of geometric information. Your can find all available factors in the "include/factors" directory. Since we are using the autodiff functionality of Ceres, they are implemented as templated function in the header files.
In the following, we list some of the available factors, but keep in mind that their number is growing and this documentation might not be complete.

### Absolute Measurements

Absolute measurements are given by sensors like GPS or radio based ranging to beacons with known positions. The corresponding **range** or **pseudorange** factors can be found in [RangeFactor.h](../include/factors/RangeFactor.h) and [PseudorangeFactor.h](../include/factors/PseudorangeFactor.h).
The pseudo-range factor is available in two variants, where `PseudorangeSagnacFactorBase` includes a relativistic correction and is made for estimation in a global ECEF system and `PseudorangeFactorBase` for calculation in a local UTM or ENU system.

A losely-coupled GPS or any kind of **prior information** can be represented by `PriorFactorBase` class from [PriorFactor.h](../include/factors/PriorFactor.h). Please note that there are special versions for different kinds of rotational representations. Please use them if you want to represent a angular value or a quaternion.

### Relative Measurements

Relative factors can be used to connect different states (variables) with relative information. Since there is a variety of implementation, we only point out the most relevant groups here.

**Constant value** and **constant drift** factors are the most basic ones, the assume that, with a given uncertainty, a value stays constant over time or moves with a constant drift. Simple motion models like constant position or constant velocity are one possible application. Tightly-coupled GPS can also implemented in conjunction with the constant drift factor as "Constant Clock Error Drift" model.
The implementation can be found in [ConstantValueFactor.h](../include/factors/ConstantValueFactor.h) and [ConstantDriftFactor.h](../include/factors/ConstantDriftFactor.h).

**Between factors** are similar to constant values factors, but assume an offset between the connected states. Different variants for vector values or geometric manifolds can be found in [BetweenValueFactor.h](../include/factors/BetweenValueFactor.h), [BetweenQuaternionFactor.h](../include/factors/BetweenQuaternionFactor.h), [BetweenPose2Factor.h](../include/factors/BetweenPose2Factor.h) and [BetweenPose3Factor.h](../include/factors/BetweenPose3Factor.h). 

Wheel based **odometry** is a typical relative sensor. The libRSF provides a broad set of different odometry factors, depending on the used state representation. They can be found in [OdometryFactor2D.h](../include/factors/OdometryFactor2D.h) and [OdometryFactor3D.h](../include/factors/OdometryFactor3D.h). In [OdometryFactor2DDifferential.h](../include/factors/OdometryFactor2DDifferential.h), a factor for a differential drive robot is provided.

Finally, we also provide a straight-forward **IMU** factor as well as a more sophisticated **IMU pre-integration** factor. They can be found in [IMUFactor.h](../include/factors/IMUFactor.h) and [IMUPreintegrationFactor.h](../include/factors/IMUPreintegrationFactor.h).

### Technical Interface

All factors have to be used in conjunction with a Error model. (For more details about error models see [here](CUSTOM_ERROR_MODELS.md).)
As the schema below shows, the output of the factor is the input of the error model. 

```
                    ------------                     -------------
   States    ----->|            |----->       ----->|             |----->
    and      ----->|   Factor   |-----> Error ----->| Error Model |-----> Weighted Error
Measurements ----->|            |----->       ----->|             |----->
                    ------------                     ------------- 	
```

To apply the automatic differentiation of Ceres, the dimensions of the weighted error has to be known at compile time and so the error model. Therefore, the error model has to be given as template argument of the factor class.
Each factor hast than a `bool operator()` function that combines the `Evaluate()` function of the factor (the deterministic model) with the `weight()` of the error model (the probabilistic model).

If you want to use our factors for your code, you have two options:

1. You use them with our `FactorGraph` class that has an dedicated `addFactor()` interface.  For examples, just have a look to the examples in the "applications" folder.

2. If you want to use them in a vanilla Ceres application you can create a cost function object with the following code snippet (example for a 2D range measurement). If you need more information about the required `Data`class, have a look at our [data structures](CUSTOM_IN_OUT.md).

   ```c++
   /** get a range measurement from somewhere */
   libRSF::Data RangeMeasurement = ...
   
   /** create error model */
   typedef libRSF::GaussianDiagonal<1> ErrorClass;
   ErrorClass NoiseModel;
   NoiseModel.setStdDevDiagonal(0.25);
   
   /** create factor object */
   typedef libRSF::RangeFactorBase<ErrorClass, 2> FactorClass;
   FactorClass* Factor = new FactorClass(NoiseModel, RangeMeasurement);
   
   /** convert it to a ceres cost function*/
   auto CostFunction = new ceres::AutoDiffCostFunction<FactorClass, ErrorClass::OutputDim, 2> (Factor);
   ```

   