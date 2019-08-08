#pragma once
#include "global_map/global_map.h"
#include "global_map/global_map_common.h"
namespace gm {
void save_submap(GlobalMap& map, std::string file_addr);
bool load_submap(GlobalMap& map, std::string file_addr, bool do_recover_obs=false);
void save_global_map(GlobalMap& map, std::string file_addr);
void load_global_map(GlobalMap& map, std::string file_addr, std::vector<unsigned int> map_ids);
}  // namespace gm
