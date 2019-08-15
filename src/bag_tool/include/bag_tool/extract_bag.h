#pragma once

#include <string>
#include <Eigen/Core>
void extract_bag(std::string out_addr_, std::string bag_addr_, std::string img_topic, std::string imu_topic, std::string gps_topic, bool isExtractImage);
void get_gps_traj(std::string bag_addr, std::string gps_topic, std::vector<Eigen::Vector3d>& gps_list,
                  std::vector<double>& gps_time_list, std::vector<int>& gps_cov_list
                 );