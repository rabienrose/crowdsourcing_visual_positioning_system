#include"global_map_api/global_map_api.h"
#include<iostream>
#include<opencv2/imgproc/imgproc.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<gflags/gflags.h>

DEFINE_string(map_addr_, "/home/meng/data/global_map/release", "");
DEFINE_string(config_root, "/home/meng/data/global_map/config", "");//camera config

int main(){

	cv::Mat img = cv::imread("/home/meng/loc/10.jpg");
	cv::Mat debug_img;
	Eigen::Matrix4d pose;
	Eigen::Vector3d gps_position;
	std::vector<cv::Point2f> inliers_kp;
	std::vector<Eigen::Vector3d> inliers_mp;
	float match_time;
	std::vector<unsigned int> map_ids = {900888464};

	gm::GlobalMapApi api;
	api.init(FLAGS_config_root, FLAGS_map_addr_);
	api.load_map(map_ids);
	bool isSuccess = api.locate_img_resize(img, debug_img, pose, gps_position, inliers_kp, inliers_mp, match_time);

	if(isSuccess){
		std::cout << "Success!" << std::endl;
		std::cout << "Position: " << pose << std::endl;
		std::cout << gps_position << std::endl;
	} else{
		std::cout << "Failed: " << std::endl;
		std::cout << inliers_mp.size() << std::endl;
		std::cout << gps_position << std::endl;
	}

	return 0;
}
