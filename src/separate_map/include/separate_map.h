#pragma once

#include <Eigen/Core>
#include <set>
#include <vector>

namespace gm{
	void get_map_block_id_from_gps(unsigned int& block_id, Eigen::Vector3d gps_latlon);

    void get_gps_from_block_id(Eigen::Vector3d& gps_latlon, const unsigned int block_id);

	std::vector<unsigned int> getNearBlock(std::vector<unsigned int>& block_ids);
}

