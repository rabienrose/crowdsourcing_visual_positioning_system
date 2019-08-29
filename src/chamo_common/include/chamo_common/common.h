#include<vector>
#include<string>
#include <Eigen/Core>


namespace chamo{
    void transformPoseUseSim3(Eigen::Matrix4d& sim3, Eigen::Matrix4d& in_pose,  Eigen::Matrix4d& out_pose);
    std::vector<std::string> split(const std::string& str, const std::string& delim);
    
}