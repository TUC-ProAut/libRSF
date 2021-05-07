## One class to store them all

To store all the data that accumulates from a variety of sensors, we created the `Data` class which is located in the [Data.h](../include/Data.h) header file.
It holds an map of (dynamically sized) vectors that store the sub-elements of a measurement. For example its mean and its variance.
The map is indexed by the `DataElement` enum, which you can find in the [Types.h](../include/Types.h) header file.

For each "kind" of measurement, there is a configuration object that initialize a specific data type. These configuration are stored in the [Types.cpp](../src/Types.cpp).
There is another enum `DataType`, which connects a Data object with it's configuration.

The `Data` class is also used to store the state variables

### Streams of Data

A `DataStream` is a chronological ordered multimap of `Data` objects. **Multi**map, because there can be multiple entries fur the same timestamp.
We use double timestamps as keys for this multimap, but round them internally to 1e-3 precision to prevent problems with floating point comparisons.

The `DataStream` class can be found in [DataStream.h](../include/DataStream.h).

### Sets of Data

Multiple `DataStream` objects can be stored in an `DataSet`. Our implementation of this `DataSet` class is again a map that contains indexed `DataStream` objects. Each stream has an unique key, but the type of the key depends on the use-case.

#### SensorDataSet

For measurements, we use `DataType` enums as Keys because we assume that each sensor is unique and connected to a specific type of data. The resulting `SensorDataSet` can be found in the [SensorDataSet.h](../include/SensorDataSet.h) header.

#### StateDataSet

For state variables of the factor graph, there is a separate `StateDataSet` class in the [StateDataSet.h](../include/StateDataSet.h) header.
Here, we use strings as keys to allow more flexibility.

The `StateDataSet` is usually used inside our `FactorGraph` class and internally connected to the Ceres back-end by raw pointers.
We created it as a more convenient interface for the optimized variables.