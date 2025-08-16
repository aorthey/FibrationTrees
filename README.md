# FibrationTrees

Light-weight frontend to create fibration trees for multi-robot motion planning problems and to visualize path planning results.

[![ContinuousIntegrationPipeline](https://github.com/aorthey/FibrationTrees/actions/workflows/pipeline.yml/badge.svg)](https://github.com/aorthey/FibrationTrees/actions/workflows/pipeline.yml)

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

Required
- [x] Parallel fibration requires a different selection method. I.e. when one
  factor is not yet solved, it should be given precedence.
- [ ] Make a unit test for parallel section search
- [ ] Make unit test for motion validation on Reedssheppcars
- [ ] Print output of benchmark to png file 
- [ ] Ensure TASK-RRT takes task-space constraints into account

Optional
- [ ] Ensure that Section Solver is first checking straight line before doing
  side steps along a fiber
- [ ] Multi robot collision checker init in MakeCollisionChecker
- [ ] Ensure that projections are sorted first before applied
- [ ] Check why task space projection takes a long time per iteration on 03multi
