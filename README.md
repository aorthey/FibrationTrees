* Dependencies
sudo apt install libdart-dev
sudo apt install libdart-external-ikfast-dev
sudo apt install libdart-imgui-dev
sudo apt install libdart-lodepng-dev


* Problems


Scenario3_MultiRobotVerticalWall: ./dart/external/imgui/imgui.cpp:7259: void ImGui::ErrorCheckEndFrameSanityChecks(): Assertion `(key_mod_flags == 0 || g.IO.KeyMods == key_mod_flags) && "Mismatching io.KeyCtrl/io.KeyShift/io.KeyAlt/io.KeySuper vs io.KeyMods"' failed.
Aborted (core dumped)

Maybe when pressing Fn/Strg/Alt/Super ?


* TODO

- [x] Ensure that multi robots can be loaded in arbitrary order
- [x] Add [x]DiskRobot, [x]SphereRobot, [x]CubeRobot to RobotLoaderTest
- [x] Add options to robot factory by using YAML::Node
- [ ] Multi robot collision checker init in MakeCollisionChecker
- [ ] Terminate immediately on invalid goal
