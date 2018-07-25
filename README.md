hpp-pinocchio
========

[![Building Status](https://travis-ci.org/humanoid-path-planner/hpp-pinocchio.svg?branch=master)](https://travis-ci.org/humanoid-path-planner/hpp-pinocchio)
[![Pipeline status](https://gepgitlab.laas.fr/humanoid-path-planner/hpp-pinocchio/badges/master/pipeline.svg)](https://gepgitlab.laas.fr/humanoid-path-planner/hpp-pinocchio/commits/master)
[![Coverage report](https://gepgitlab.laas.fr/humanoid-path-planner/hpp-pinocchio/badges/master/coverage.svg?job=doc-coverage)](http://projects.laas.fr/gepetto/doc/humanoid-path-planner/hpp-pinocchio/master/coverage/)

This package implements a library that wraps the [Pinocchio](http://stack-of-tasks.github.io/pinocchio/) library to be used in the [Humanoid Path Planner](https://humanoid-path-planner.github.io/hpp-doc/) framework.

Setup
-----

The easiest way to install this package is to uses the [installation instructions of HPP](https://humanoid-path-planner.github.io/hpp-doc/download.html).
For a manual installation, follow these steps.

To compile this package, it is recommended to create a separate build
directory:

    mkdir _build
    cd _build
    cmake [OPTIONS] ..
    make install

Please note that CMake produces a `CMakeCache.txt` file which should
be deleted to reconfigure a package from scratch.


### Dependencies

The package depends on several packages which have to be available on
your machine.

 - Libraries:
   - Boost (>=1.48.0)
     Boost Test is used in the test suite
   - Pinochio (>=1.1)
 - System tools:
   - CMake (>=2.6)
   - pkg-config
   - usual compilation tools (GCC/G++, make, etc.)
