#pragma once

#include <string>
#include <vector>
#include <gtest/gtest.h>

std::vector<std::string> GetFilesRecursively(const std::string& path);

std::string FileStemFromParam(const testing::TestParamInfo<std::string>& info);
std::string FileStemFromParamPairStringDouble(const testing::TestParamInfo<std::pair<std::string, double>>& info);
