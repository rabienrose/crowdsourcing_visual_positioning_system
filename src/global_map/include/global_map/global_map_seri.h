#pragma once
#include "global_map/global_map.h"
#include "global_map/global_map_common.h"
namespace gm {
void save_submap(GlobalMap& map, std::string file_addr);
bool load_submap(GlobalMap& map, std::string file_addr, bool do_recover_obs=false);
void save_global_map(GlobalMap& map, std::string file_addr);
void load_global_map(GlobalMap& map, std::string file_addr, std::vector<unsigned int> map_ids);
void load_global_map_by_gps(GlobalMap& map, std::string file_addr, Eigen::Vector3d gps_position);
void get_blockid_list(GlobalMap& map, std::vector<unsigned int>& out_block_ids);
void fast_load_mps(std::vector<Eigen::Vector3d>& posis, std::string file_addr, std::vector<unsigned int> ids);
}  // namespace gm
