#pragma once
#include "global_map/global_map.h"
#include "global_map/global_map_common.h"
namespace gm {
void save_global_map(GlobalMap& map, std::string file_addr);
bool loader_global_map(GlobalMap& map, std::string file_addr);
}  // namespace gm
