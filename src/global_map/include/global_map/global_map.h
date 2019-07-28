#pragma once
#include "global_map/frame.h"
#include "global_map/map_point.h"
#include "global_map/global_map_common.h"

namespace gm {
class GlobalMap {
public:
    std::vector<std::shared_ptr<Frame>> frames;
    std::vector<std::shared_ptr<MapPoint>> mappoints;
    int map_id;
    Eigen::Vector3d gps_anchor;
    std::vector<std::shared_ptr<Frame>> pose_graph_v1;
    std::vector<std::shared_ptr<Frame>> pose_graph_v2;
    std::vector<Eigen::Vector3d> pose_graph_e_posi;
    std::vector<Eigen::Matrix3d> pose_graph_e_rot;
    std::vector<double> pose_graph_e_scale;
    std::vector<double> pose_graph_weight;
    Eigen::Vector3d Tbc_posi;
    Eigen::Quaterniond Tbc_qua;

    void CreateSubMap(int startframe_id, int endframe_id, GlobalMap& submap);
    void ComputeUniqueId();
    void DelMappoint(int id);
    void DelFrame(int id);
    void GetMPPosiList(std::vector<Eigen::Vector3d>& mp_posis);
    std::shared_ptr<MapPoint> getMPById(int id);
    void CheckConsistence();
    void GetCovisi(std::shared_ptr<Frame> frame_p, std::map<std::shared_ptr< Frame>, int>& connections);
    void AssignKpToMp();
    void CalPoseEdgeVal();
    void UpdatePoseEdge();
    void AddConnection(std::shared_ptr<Frame> v1,
                       std::shared_ptr<Frame> v2,
                       Eigen::Vector3d& posi,
                       Eigen::Matrix3d& rot,
                       double scale,
                       double weight);
    void FilterTrack(); //remove the track that connect to the same frame. Keep the one with smallest reprojection error.
};
}  // namespace gm
