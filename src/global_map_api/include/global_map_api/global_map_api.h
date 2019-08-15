#pragma once
#include <string>
#include <fstream>
#include <memory>
#include <vector>
#include <opencv2/core/core.hpp>
#include <Eigen/Core>

namespace ORB_SLAM2{
    class ORBextractor;
}

namespace chamo{
    class GlobalMatch;
}

namespace gm{
    class GlobalMap;
    class GlobalMapApi{
    public:
        bool init(std::string config_addr_, std::string map_addr_);
        bool load_map(std::vector<Eigen::Vector3d> gps_positions);
        bool get_pointcloud(std::vector<Eigen::Vector3d>& out_pointcloud);
        bool process_bag(std::string bag_addr, std::string cache_addr_, std::string localmap_addr);
        bool locate_img(cv::Mat img, Eigen::Matrix4d& pose, Eigen::Vector3d gps_position,
            std::vector<cv::Point2f>& inliers_kp, std::vector<int>& inliers_mp
        );
    private:
        std::string config_addr;
        std::string map_addr;
        std::shared_ptr<GlobalMap> cur_map;
        std::shared_ptr<ORB_SLAM2::ORBextractor> extractor;
        std::shared_ptr<chamo::GlobalMatch> matcher;
        double fx, fy, cx, cy;
        double k1, k2, p1, p2;
        
        
    };
}
