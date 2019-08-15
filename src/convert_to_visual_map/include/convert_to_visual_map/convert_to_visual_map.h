#pragma once
#include <string>
#include <fstream>
#include <memory>
#include <vector>

void convert_to_visual_map(std::string config_root, std::string res_root, std::string globalmap_root, std::vector<unsigned int>& ids);