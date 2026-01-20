# FibrationTrees

Light-weight frontend to create fibration trees for multi-robot motion planning problems and to visualize path planning results.

[![ContinuousIntegrationPipeline](https://github.com/aorthey/FibrationTrees/actions/workflows/pipeline.yml/badge.svg)](https://github.com/aorthey/FibrationTrees/actions/workflows/pipeline.yml)

# Installation

Required packages for Ubuntu 22.04:
```
sudo apt install libboost-filesystem-dev \
        libboost-program-options-dev \
        libboost-serialization-dev \
        libboost-system-dev \
        libboost-test-dev \
        libeigen3-dev \
        libflann-dev \
        libode-dev \
        libtriangle-dev \
        libyaml-cpp-dev \
        libdart-all-dev \
        libdart-gui-osg-dev \
        libdart-utils-dev \
        libdart-utils-urdf-dev \
        libdart-collision-bullet-dev \
        libdart-external-ikfast-dev \
        libdart-external-imgui-dev \
        libdart-external-lodepng-dev \
        libgtest-dev
```

# Usage

There are 12 possible scenarios which you can run, which are located in
`data/scenarios/`. You can run each scenario as a standalone by using the
`executables/FibrationTreesSolver.cpp` script or run the complete benchmarks in
`executables/FibrationTreesBenchmaker.cpp`.
