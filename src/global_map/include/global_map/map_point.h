#pragma once
#include "global_map/global_map_common.h"

namespace gm {
class Frame;
class TrackItem {
public:
    std::shared_ptr<Frame> frame;
    int kp_ind;
    void getUV(float& x, float& y, int& octave);
};

class MapPoint {
public:
    MapPoint();
    long unsigned int id = -1;
    bool isfix;
    int match_count=0;
    Eigen::Vector3d position;
    std::vector<TrackItem> track;
    int calDescDiff(Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic>& query_desc);
    void getALLDesc(Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic>& descs);
};

}  // namespace gm
