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

- [ ] Ensure that multi robots can be loaded in arbitrary order
