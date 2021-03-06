# A Linear-Time Variational Integrator for Multibody Systems

[![Build Status](https://travis-ci.org/jslee02/wafr2016.svg?branch=master)](https://travis-ci.org/jslee02/wafr2016)

:warning: This repository is under development. There is still room for improvement in terms of API design or code optimization. Also, the quantitative of the performance tests can be different from the paper, but the qualitative performance is the same.

This repository contains the algorithms introduced by "A Linear-Time Variational Integrator for Multibody Systems" and several state-of-art algorithms that are implemented on top of [DART](http://dartsim.github.io/) v6.0.0.

### Implemented Algorithms

* Root finding algorithms to solve the DEL equation
  * Newton's method
  * Secant method
  * Broyden method (used [GSL](https://www.gnu.org/software/gsl/))
  * Recursive Impluse-based Quasi-Newton (RIQN) method [1]
* Recusive algorithms to evaluate the DEL equation
  * Scalable Variational Integrator (SVI) [2]
  * Discrete Recursive Newton-Euler Algorithm (DRNEA) [1]

### Results

TODO

### How to Build

#### On Ubuntu

**Install Dependencies**
```
$ sudo apt install cmake libassimp-dev libboost-all-dev libccd-dev libeigen3-dev libfcl-dev freeglut3-dev libxi-dev libxmu-dev libflann-dev libnlopt-dev libtinyxml-dev libtinyxml2-dev liburdfdom-dev liburdfdom-headers-dev libopenscenegraph-dev libgsl0-dev
```

**Build**
```
$ cd <wafr2016_root_dir>
$ mkdir build
$ cd build
$ cmake .. -DCMAKE_BUILD_TYPE=Release
$ make -j experiments examples
```

**Running Example**
```
$ cd <wafr2016_root_dir>/build/bin
$ ./dm_passivePendulum
```

**Running Experiments**
```
$ cd <wafr2016_root_dir>/build/bin
$ ./energy_conservation
$ ./convergence
$ ./iteration_numbers
$ ./performance_comparisons
```

### References

[1] Jeongseok Lee, C. Karen Liu, Frank C. Park, and Siddhartha S. Srinivasa, “**A Linear-Time Variational Integrator for Multibody Systems**,” in the International Workshop on the Algorithmic Foundations of Robotics, 2016 (accepted) [[arXiv](https://arxiv.org/abs/1609.02898)]

[2] Elliot R. Johnson, Todd D. Murphey, "**Scalable variational integrators for constrained mechanical systems in generalized coordinates**," IEEE Transactions on Robotics, 25(6) (2009) 1249-1261

