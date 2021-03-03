## One class to store them all

-  purpose
   -  store (relativly) big a mount of heterogenous data
- general concept of data
  - a map of elementIDs and vectors
  - acces through setter and getters
  - config define the structure --> point to config
- general concept of dataset
  - multimaps inside a map
  - first layer is ID
  - second layer is time
  - double for time is dangerous, keep in mind

### Measurement Data

* store measurements
* type as ID

### State Data

* store optimized values for ceres pointer interface
* different states with the same type --> string as ID

