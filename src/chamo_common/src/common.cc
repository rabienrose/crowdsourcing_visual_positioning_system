#include<chamo_common/common.h>
#include <Eigen/Core>
namespace chamo{
    std::vector<std::string> split(const std::string& str, const std::string& delim)
    {
        std::vector<std::string> tokens;
        size_t prev = 0, pos = 0;
        do
        {
            pos = str.find(delim, prev);
            if (pos == std::string::npos) pos = str.length();
            std::string token = str.substr(prev, pos-prev);
            if (!token.empty()) tokens.push_back(token);
            prev = pos + delim.length();
        }
        while (pos < str.length() && prev < str.length());
        return tokens;
    }
    
    void transformPoseUseSim3(Eigen::Matrix4d& sim3, Eigen::Matrix4d& in_pose,  Eigen::Matrix4d& out_pose){
        Eigen::Matrix3d R_tran=sim3.block(0,0,3,3);
        double scale=R_tran.block(0,0,3,1).norm();
        R_tran=R_tran/scale;
        Eigen::Matrix3d R_in=in_pose.block(0,0,3,3);
        Eigen::Matrix3d R_out=R_tran*R_in;
        Eigen::Vector4d t_out=sim3*in_pose.block(0,3,4,1);
        out_pose= Eigen::Matrix4d::Identity();
        out_pose.block(0,0,3,3) = R_out;
        out_pose.block(0,3,4,1) = t_out;
    }
}


