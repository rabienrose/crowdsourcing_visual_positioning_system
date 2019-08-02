#ifndef SIM3_MATCH_H
#define SIM3_MATCH_H

#include <opencv2/opencv.hpp>
#include <Eigen/Core>

namespace chamo
{
    void ComputeCentroid(cv::Mat &P, cv::Mat &Pr, cv::Mat &C);
    void ComputeSim3(cv::Mat &P1, cv::Mat &P2, cv::Mat& mT12i, double& scale);
    void ComputeSim3(std::vector<Eigen::Vector3d>& P1, std::vector<Eigen::Vector3d>& P2, Eigen::Matrix4d& T12i_eig, double& scale);
    bool ComputeSim3Ransac(std::vector<Eigen::Vector3d>& P1, std::vector<Eigen::Vector3d>& P2, Eigen::Matrix4d& T12, double& scale_12);
}


#endif