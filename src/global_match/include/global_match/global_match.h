#pragma once

#include <Eigen/Dense>
#include <opencv2/core/core.hpp>
#include <inverted-multi-index/inverted-multi-index.h>
#include "descriptor-projection/build-projection-matrix.h"
#include "global_map/global_map.h"

namespace chamo {
class GlobalMatch{
public:
    void LoadMap(std::string project_mat_file, gm::GlobalMap& _map, Eigen::Vector3d est_posi);

    void MatchImg(std::shared_ptr<gm::Frame> query_frame, std::vector<std::vector<int>>& inliers_mps, 
                                std::vector<std::vector<int>>& inliers_kps, std::vector<Eigen::Matrix4d>& poses,
                                float project_err_range=20, float desc_diff_err=50, bool do_reproj=true
    );
    
    std::shared_ptr<loop_closure::inverted_multi_index::InvertedMultiIndex<5>> index_;
    Eigen::MatrixXf projection_matrix_;
    gm::GlobalMap map;
};


}