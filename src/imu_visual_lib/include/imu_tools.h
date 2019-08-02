#ifndef IMU_TOOLS_H
#define IMU_TOOLS_H

#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include <IMUPreintegrator.h>
#include <IMUPreintegrator.h>
#include <NavState.h>
namespace chamo
{
    struct MP_INFO{
    public:
        float u;
        float v;
        int octove;
        int frame_id;
        int mp_id;
    };
    
    Eigen::Matrix3d calRotMFromGravity(Eigen::Vector3d gravity);
    
    Eigen::Vector3d OptimizeInitialGyroBias(const std::vector<cv::Mat>& vTwc, const std::vector<IMUPreintegrator>& vImuPreInt,
        Matrix4d Tbc
    );
    
    void GlobalBundleAdjustmentNavStatePRV(std::vector<IMUPreintegrator>& preints, std::vector<NavState>& states, Matrix4d Tbc,
                                       std::vector<std::vector<MP_INFO>> mp_infos, float fx, float fy, float cx, float cy,
                                       std::vector<Eigen::Vector3d>& mp_posis,
                                       const cv::Mat& gw, int nIterations);

    void CalAccBias(const std::vector<cv::Mat>& vTwc, const std::vector<IMUPreintegrator>& vImuPreInt, 
                    double& sstar, cv::Mat& gwstar, cv::Mat Tbc, Eigen::Matrix3d& Rwi, Eigen::Vector3d& bias_a);
    void CalGravityAndScale(const std::vector<cv::Mat>& vTwc, const std::vector<IMUPreintegrator>& vImuPreInt, cv::Mat Tbc,
        double& sstar, cv::Mat& gwstar, double& scale_confi, double& grav_confi);
}
#endif // CONFIGPARAM_H
