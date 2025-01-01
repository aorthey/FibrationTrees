# FibrationTrees

Light-weight frontend to create fibration trees for different motion planning problems.

[![ContinuousIntegrationPipeline](https://github.com/aorthey/FibrationTrees/actions/workflows/c-cpp.yml/badge.svg)](https://github.com/aorthey/FibrationTrees/actions/workflows/c-cpp.yml)

# Dependencies
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

# Known Problems

Scenario3_MultiRobotVerticalWall: ./dart/external/imgui/imgui.cpp:7259: void ImGui::ErrorCheckEndFrameSanityChecks(): Assertion `(key_mod_flags == 0 || g.IO.KeyMods == key_mod_flags) && "Mismatching io.KeyCtrl/io.KeyShift/io.KeyAlt/io.KeySuper vs io.KeyMods"' failed.
Aborted (core dumped)

Maybe when pressing Fn/Strg/Alt/Super ?

#  TODO

- [x] Ensure that multi robots can be loaded in arbitrary order
- [x] Add [x]DiskRobot, [x]SphereRobot, [x]CubeRobot to RobotLoaderTest
- [x] Add options to robot factory by using YAML::Node
- [x] Fix workflow to execute tests on github
- [x] Add CubeRobot DvP scenario
- [ ] Add MobileManip DvP scenario
- [ ] Multi robot collision checker init in MakeCollisionChecker
- [ ] Terminate immediately on invalid goal
