#include "global_map/frame.h"

namespace gm {
Frame::Frame()
{
    imu_next_frame = nullptr;
    gps_avg_count=0;
    isfix=false;
    doBA=false;
    doGraphOpti=false;
    doMatch=false;
    isborder=false;
    willDel=false;
}
Eigen::Matrix4d Frame::getPose()
{
    Eigen::Matrix3d temp_rot(direction);
    Eigen::Matrix4d temp_pose = Eigen::Matrix4d::Identity();
    temp_pose.block(0, 0, 3, 3) = temp_rot;
    temp_pose.block(0, 3, 3, 1) = position;
    return temp_pose;
}

void Frame::setPose(Eigen::Matrix4d pose)
{
    position = pose.block(0, 3, 3, 1);
    Eigen::Matrix3d tempRot = pose.block(0, 0, 3, 3);
    Eigen::Quaterniond temp_qua(tempRot);
    direction = temp_qua;
}

Eigen::Matrix<double, 3, 4> Frame::getProjMat()
{
    Eigen::Matrix<double, 3, 4> k_mat = Eigen::Matrix<double, 3, 4>::Zero();
    k_mat(0, 0) = fx;
    k_mat(1, 1) = fy;
    k_mat(0, 2) = cx;
    k_mat(1, 2) = cy;
    k_mat(2, 2) = 1;
    Eigen::Matrix<double, 3, 4> proj = k_mat * getPose().inverse();
    return proj;
}

Eigen::Matrix<double, 3, 3> Frame::getKMat()
{
    Eigen::Matrix<double, 3, 3> k_mat = Eigen::Matrix<double, 3, 3>::Identity();
    k_mat(0, 0) = fx;
    k_mat(1, 1) = fy;
    k_mat(0, 2) = cx;
    k_mat(1, 2) = cy;
    return k_mat;
}

void Frame::getDesc(int ind, Eigen::Matrix<unsigned char, Eigen::Dynamic, 1>& desc_out)
{
    if (ind >= descriptors.cols()) {
        std::cout << "[Frame::getDesc][error]ind>=descriptors.cols()" << std::endl;
        exit(0);
    }
    desc_out = descriptors.col(ind);
}

void Frame::AddKPAndDesc(cv::KeyPoint kp,
                         Eigen::Matrix<unsigned char, Eigen::Dynamic, 1>& desc,
                         std::shared_ptr<MapPoint> mp)
{
    descriptors.conservativeResize(desc.rows(), descriptors.cols() + 1);
    descriptors.col(descriptors.cols() - 1) = desc;
    obss.push_back(mp);
    kps.push_back(kp);
}

}  // namespace gm
