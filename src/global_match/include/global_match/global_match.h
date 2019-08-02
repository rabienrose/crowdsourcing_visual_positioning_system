#pragma once

#include <Eigen/Dense>
#include <opencv2/core/core.hpp>
#include <inverted-multi-index/inverted-multi-index.h>
#include "descriptor-projection/build-projection-matrix.h"
#include "global_map/global_map.h"

namespace chamo {
class GlobalMatch{
public:
    void LoadMap(std::string project_mat_file, std::shared_ptr<gm::GlobalMap> map);

    void MatchImg(std::shared_ptr<gm::Frame> query_frame, std::vector<std::vector<int>>& inliers_mps, 
                                std::vector<std::vector<int>>& inliers_kps, std::vector<Eigen::Matrix4d>& poses
    );
    
    std::shared_ptr<loop_closure::inverted_multi_index::InvertedMultiIndex<5>> index_;
    Eigen::MatrixXf projection_matrix_;
    std::shared_ptr<gm::GlobalMap> map_p;
};


}