#pragma once

#include "global_map/global_map_common.h"
namespace gm {
class MapPoint;
class Frame {
public:
    Frame();
    long unsigned int id;
    bool isfix;
    double time_stamp;
    std::string frame_file_name;
    std::vector<cv::KeyPoint> kps;
    Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic> descriptors;
    std::vector<std::shared_ptr<MapPoint>> obss;
    std::vector<long unsigned int> obss_ids; //temp variable used in loading data
    std::vector<Eigen::Vector3d> acces;
    std::vector<Eigen::Vector3d> gyros;
    std::vector<double> imu_times;
    std::shared_ptr<Frame> imu_next_frame;
    long unsigned int imu_next_frame_id; //temp variable used in loading data
    Eigen::Vector3d position;
    Eigen::Quaterniond direction;
    Eigen::Vector3d gps_position;
    float gps_accu;
    int gps_avg_count;
    
    Eigen::Vector3d Tbc_posi;
    Eigen::Quaterniond Tbc_qua;

    float fx;
    float fy;
    float cx;
    float cy;
    float k1;  // radian
    float k2;  // radian
    float p1;  // tan
    float p2;  // tan
    int width;
    int height;

    Eigen::Matrix4d getPose();
    void setPose(Eigen::Matrix4d pose);

    Eigen::Matrix<double, 3, 3> getKMat();
    Eigen::Matrix<double, 3, 4> getProjMat();

    void getDesc(int ind, Eigen::Matrix<unsigned char, Eigen::Dynamic, 1>& desc_out);
    void AddKPAndDesc(cv::KeyPoint kp,
                      Eigen::Matrix<unsigned char, Eigen::Dynamic, 1>& desc,
                      std::shared_ptr<MapPoint> mp);
    void LoadDesc(std::string desc_file);
};
}  // namespace gm
