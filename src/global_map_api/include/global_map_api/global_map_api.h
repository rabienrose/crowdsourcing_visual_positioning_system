#pragma once
#include <string>
#include <fstream>
#include <memory>
#include <vector>
#include <opencv2/core/core.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>

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
        GlobalMapApi();
        bool init(std::string config_addr_, std::string map_addr_);
        bool load_map(std::vector<unsigned int> map_ids);
        bool get_pointcloud(std::vector<Eigen::Vector3d>& out_pointcloud, std::vector<Eigen::Vector3d>& kf_posis, std::vector<Eigen::Quaterniond>& kf_rot, std::vector<unsigned int> ids);
        bool final_proc(std::string in_addr,std::string final_addr, std::vector<unsigned int> block_ids);
        bool process_bag(std::string bag_addr, std::string cache_addr_, std::string localmap_addr, std::string& status);
        bool process_bag_orb(std::string bag_addr, std::string cache_addr, std::string localmap_addr, std::string& status);
        bool locate_img(cv::Mat img, cv::Mat& debug_img, Eigen::Matrix4d& pose, Eigen::Vector3d gps_position,
                        std::vector<cv::Point2f>& inliers_kp, std::vector<Eigen::Vector3d>& inliers_mp, float& match_time
        );
        bool get_mpkf_count(int& mp_count, int& kf_count, std::vector<unsigned int> ids);
        void Release();
        bool map_is_change;
        bool match_is_change;
        bool img_is_change;
        bool reproj_err_is_change;
        std::vector<Eigen::Vector3d> debug_mp_posi;
        std::vector<Eigen::Vector3d> debug_kf_posi;
        std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> debug_matches;
        cv::Mat debug_img;
        float debug_reproj_err;

    private:
        void vis_map(gm::GlobalMap& map);
        void do_vslam(std::string local_addr, std::string config_addr, std::string bag_addr, std::string& status);
        std::string config_addr;
        std::string map_addr;
        std::shared_ptr<GlobalMap> cur_map;
        std::shared_ptr<ORB_SLAM2::ORBextractor> extractor;
        std::shared_ptr<chamo::GlobalMatch> matcher;
        
        double fx, fy, cx, cy;
        double k1, k2, p1, p2;
        cv::Mat map1,map2;

        
    };
}
