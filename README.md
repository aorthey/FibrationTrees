# FibrationTrees

Light-weight frontend to create fibration trees for multi-robot motion planning problems and to visualize path planning results.

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
- [x] Fix bug on XR3R2SO2 XR3
- [x] Add MobileManip DvP scenario

Required
- [ ] Save and Load Active View
- [ ] Terminate immediately on invalid goal
- [x] Parallel fibration requires a different selection method. I.e. when one
  factor is not yet solved, it should be given precedence.
- [ ] Make a unit test for parallel section search
- [ ] Make unit test for motion validation on Reedssheppcars

Optional
- [ ] Rename FactorTrees branch ompl to Fibrationtrees 
- [ ] Ensure that Section Solver is first checking straight line before doing
  side steps along a fiber
- [ ] Multi robot collision checker init in MakeCollisionChecker
- [ ] Ensure that projections are sorted first before applied
- [ ] Check why task space projection takes a long time per iteration on 03multi

# Scenarios

MultiRobot-Easy
 DONE PLAN BENCH
- [x]  [x]  [ ]  MultiDisks
- [x]  [x]  [ ]  Drones
- [x]  [ ]  [ ]  MobileNavigation [Needs to find solutions < 720s!]
- [x]  [x]  [ ]  ReedsSheppCars

Multirobot-hard
- [x]  [x]  [ ]  CubeRobots
- [ ]  [ ]  [ ]  Drones with Different Sizes
- [ ]  [ ]  [ ]  Mobile Manips in Cubic Forest?
- [ ]  [ ]  [ ]  ReedsSheppCars Complex

Task-space
- [x]  [x]  [ ]  Vertical-Maze
- [x]  [x]  [ ]  MultiRobot-VerticalWall [Needs a solution < 720s! -> Large variation, but seems fine]
- [x]  [x]  [ ]  MobileManipulators
- [x]  [x]  [ ]  PathVelocityDecomposition
