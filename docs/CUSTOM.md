## Learn about the libRSF
The libRSF contains of a variety of functionalities – Cost functions for for different types of measurements, robust error models and powerful data structures to handle big estimation problems.
We provide some example applications, but if you want to build your own or if you just want to use one of these components, please have a look at the topics:

[How to use the our cost functions?](CUSTOM_FACTORS.md)

[How to use our robust error models?](CUSTOM_ERROR_MODELS.md)

[How to use our data structures?](CUSTOM_IN_OUT.md)

## CMake
We provide a minimal example CMakeList.txt to include the libRSF into your CMake project:

```CMake
cmake_minimum_required(VERSION 3.5)
project(your_project)

find_package(libRSF REQUIRED)

add_executable(YourApplication YourApplication.cpp)
target_link_libraries(YourApplication libRSF::libRSF)
```
