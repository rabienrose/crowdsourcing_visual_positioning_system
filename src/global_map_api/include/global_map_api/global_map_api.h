#include <string>
#include <fstream>
#include <memory>

namespace gm{
    class GlobalMap;
    class global_map_api{
    public:
        bool init(std::string config_addr_, std::string map_addr_, std::string cache_addr_);
        bool load_map(Eigen::Vector3d gps_position);
        bool get_pointcloud(std::vector<Eigen::Vector3d>& out_pointcloud);
        bool process_bag(std::string bag_addr);
        bool locate_img(cv::Mat img, Eigen::Matrix4d, Eigen::Vector3d gps_position);
    private:
        std::string config_addr;
        std::string cache_addr;
        std::string map_addr;
        std::shared_ptr<GlobalMap> cur_map;
        double fx, fy, cx, cy;
        double k1, k2, p1, p2;
        
        
    };
}
