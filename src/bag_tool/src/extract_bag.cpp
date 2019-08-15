#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <cstdio>
#include <iostream>
#include <string>
#include <sstream>
#include <iostream>
#include <fstream>
#include <map>
#include <opencv2/opencv.hpp>
#include <memory>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CompressedImage.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <bag_tool/extract_bag.h>
#include "CoorConv.h"
#include <Eigen/Core>
#include <gflags/gflags.h>
DEFINE_double(gps_anchor_x, -1, "");
DEFINE_double(gps_anchor_y, -1, "");
DEFINE_double(gps_anchor_z, -1, "");

void findNearGPS(int& gps1, int& gps2, std::vector<double>& gps_times, double frame_time){
    gps1=-1;
    gps2=-1;
    for(int i=0; i<gps_times.size() ; i++){
        if(gps_times[i]>frame_time){
            if(i==0){
                return;
            }
            if(frame_time - gps_times[i-1]>2){
                return;
            }
            if(gps_times[i] - frame_time >2){
                return;
            }
            gps1=i-1;
            gps2=i;
            return;
        }
    }
}

void interDouble(double v1, double v2, double t1, double t2, double& v3_out, double t3){
    v3_out=v1+(v2-v1)*(t3-t1)/(t2-t1);
}

void get_gps_traj(std::string bag_addr, std::string gps_topic, std::vector<Eigen::Vector3d>& gps_list,
                  std::vector<double>& gps_time_list, std::vector<int>& gps_cov_list
                 ){
    rosbag::Bag bag;
    bag.open(bag_addr,rosbag::bagmode::Read);
    std::vector<std::string> topics;
    topics.push_back(gps_topic);
    rosbag::View view(bag, rosbag::TopicQuery(topics));
    rosbag::View::iterator it= view.begin();
    for(;it!=view.end();it++){
        rosbag::MessageInstance m =*it;
        sensor_msgs::NavSatFixPtr sgps = m.instantiate<sensor_msgs::NavSatFix>();
        if(sgps!=NULL){
            double sec = sgps->header.stamp.toSec();
            Eigen::Vector3d gps_data(sgps->latitude, sgps->longitude, sgps->altitude);
            gps_list.push_back(gps_data);
            gps_time_list.push_back(sec);
            gps_cov_list.push_back((int)sgps->position_covariance[0]);
        }
    }
}

void extract_bag(std::string out_addr_, std::string bag_addr_, std::string img_topic, std::string imu_topic, std::string gps_topic, bool isExtractImage){
    
    std::string bag_addr=bag_addr_;
    std::string out_dir=out_addr_;
    rosbag::Bag bag;
    bag.open(bag_addr,rosbag::bagmode::Read);

    std::vector<std::string> topics;
    topics.push_back(img_topic);
    topics.push_back(imu_topic);
    topics.push_back(gps_topic);
    rosbag::View view(bag, rosbag::TopicQuery(topics));
    int img_count=0;
    rosbag::View::iterator it= view.begin();
    std::ofstream outfile_img_time;
    outfile_img_time.open (out_dir+"/image_time.txt");
    
    std::ofstream outfile_imu;
    outfile_imu.open (out_dir+"/imu.txt");
    
    std::ofstream outfile_gps;
    outfile_gps.open (out_dir+"/gps.txt");
    
    std::ofstream outfile_gps_orth;
    outfile_gps_orth.open (out_dir+"/gps_orth.txt");
    int gps_count=0;
    Eigen::Vector3d anchorGps;
    std::vector<double> img_timess;
    std::vector<double> gps_times_hcov;
    std::vector<std::string> img_names;
    std::vector<Eigen::Vector3d> gps_orths_hcov;
    std::vector<double> gps_confid;
    std::vector<double> new_gps_confid;
    for(;it!=view.end();it++){
        
        rosbag::MessageInstance m =*it;

        sensor_msgs::CompressedImagePtr simg = m.instantiate<sensor_msgs::CompressedImage>();
        if( simg!=NULL ){
            cv_bridge::CvImagePtr cv_ptr;
            try{
                if(isExtractImage ){
                    cv_ptr = cv_bridge::toCvCopy(simg, "bgr8");
                    std::stringstream ss;
                    ss<<out_dir+"/images/img_"<<img_count<<".jpg";
                    cv::imwrite(ss.str(), cv_ptr->image);
                }std::stringstream ss_name;
                    ss_name<<"img_"<<img_count<<".jpg";
                std::stringstream ss_time;
                ss_time<<"img_"<<img_count<<".jpg"<<","<<simg->header.stamp<<std::endl;
                outfile_img_time<<ss_time.str();
                img_timess.push_back(simg->header.stamp.toSec());
                img_names.push_back(ss_name.str());
            }catch (cv_bridge::Exception& e){
                ROS_ERROR("cv_bridge exception: %s", e.what());
                return;
            }
            img_count++;
        }
        
        sensor_msgs::ImuPtr simu = m.instantiate<sensor_msgs::Imu>();
        if(simu!=NULL){
            double sec = simu->header.stamp.toSec();
            std::stringstream ss;
            ss<<std::setprecision (15)<<sec<<","<<simu->angular_velocity.x<<","<<simu->angular_velocity.y<<","<<simu->angular_velocity.z<<","<<simu->linear_acceleration.x<<","<<simu->linear_acceleration.y<<","<<simu->linear_acceleration.z<<std::endl;
            outfile_imu<<ss.str();
        }
        
        sensor_msgs::NavSatFixPtr sgps = m.instantiate<sensor_msgs::NavSatFix>();
        if(sgps!=NULL){
            double sec = sgps->header.stamp.toSec();
            std::stringstream ss;
            ss<<std::setprecision (15)<<sec<<","<<sgps->latitude<<","<<sgps->longitude<<","<<sgps->altitude<<","<<(int)sgps->position_covariance[0]<<std::endl;
            outfile_gps<<ss.str();
            Eigen::Vector3d coor_gps;
            Eigen::Vector3d ori_gps;
            ori_gps(0)=sgps->latitude;
            ori_gps(1)=sgps->longitude;
            ori_gps(2)=sgps->altitude;
            if(gps_count==0){
                if(FLAGS_gps_anchor_x==-1 && FLAGS_gps_anchor_y==-1 && FLAGS_gps_anchor_z==-1){
                    anchorGps=ori_gps;
                }else{
                    anchorGps<<FLAGS_gps_anchor_x, FLAGS_gps_anchor_y, FLAGS_gps_anchor_z; 
                }
                
                std::stringstream ss1;
                ss1<<std::setprecision (15)<<anchorGps(0)<<","<<anchorGps(1)<<","<<anchorGps(2)<<std::endl; 
                outfile_gps_orth<<ss1.str();
            }
            gps_count++;
            convert_to_coor(ori_gps, coor_gps, anchorGps);
            std::stringstream ss1;
            ss1<<std::setprecision (15)<<sec<<","<<coor_gps(0)<<","<<coor_gps(1)<<","<<coor_gps(2)<<","<<(int)sgps->position_covariance[0]<<std::endl; 
            outfile_gps_orth<<ss1.str();
            gps_times_hcov.push_back(sec);
            gps_orths_hcov.push_back(coor_gps);
            if(sgps->status.status==sensor_msgs::NavSatStatus::STATUS_FIX){
                gps_confid.push_back(0.05);
                //gps_confid.push_back(sgps->position_covariance[0]);
            }else{
                gps_confid.push_back(100);
            }
            
        }
    }
    outfile_img_time.close();
    outfile_imu.close();
    outfile_gps.close();
    outfile_gps_orth.close();
    
    std::vector<int> img_to_gps_ids;
    std::vector<Eigen::Vector3d> img_gpss;
    for (int i=0; i<img_timess.size(); i++){
        int gps1;
        int gps2;
        findNearGPS(gps1, gps2, gps_times_hcov, img_timess[i]);
        
        if(gps1== -1){
            img_to_gps_ids.push_back(-1);
            continue;
        }
        double i_gps_x;
        double i_gps_y;
        double i_gps_z;
        double inter_confid;
        interDouble(gps_orths_hcov[gps1](0), gps_orths_hcov[gps2](0), gps_times_hcov[gps1], gps_times_hcov[gps2], i_gps_x, img_timess[i]);
        interDouble(gps_orths_hcov[gps1](1), gps_orths_hcov[gps2](1), gps_times_hcov[gps1], gps_times_hcov[gps2], i_gps_y, img_timess[i]);
        interDouble(gps_orths_hcov[gps1](2), gps_orths_hcov[gps2](2), gps_times_hcov[gps1], gps_times_hcov[gps2], i_gps_z, img_timess[i]);
        interDouble(gps_confid[gps1], gps_confid[gps2], gps_times_hcov[gps1], gps_times_hcov[gps2], inter_confid, img_timess[i]);
        
        Eigen::Vector3d new_gps_frame;
        new_gps_frame(0)=i_gps_x;
        new_gps_frame(1)=i_gps_y;
        new_gps_frame(2)=i_gps_z;
        Eigen::Vector3d latlon_gps;
        convert_to_lonlat(new_gps_frame, latlon_gps, anchorGps);
        img_gpss.push_back(latlon_gps);
        new_gps_confid.push_back(inter_confid);
        img_to_gps_ids.push_back(img_gpss.size()-1);
    }
    std::string gps_out_addr=out_dir+"/gps_alin.txt";
    std::ofstream outfile_gps_align;
    outfile_gps_align.open(gps_out_addr.c_str());
    for(int i=0; i<img_timess.size(); i++){
        int gps_id = img_to_gps_ids[i];
        if(gps_id==-1){
            outfile_gps_align<<img_names[i]<<","<<"-1"
            <<std::endl;
        }else{
            Eigen::Vector3d gps_temp= img_gpss[gps_id];
            outfile_gps_align<<std::setprecision (15)<<img_names[i]<<","<<i
            <<","<<gps_temp(0)<<","<<gps_temp(1)<<","<<gps_temp(2)<<","<<new_gps_confid[gps_id]<<std::endl;
        }
    }
    outfile_gps_align.close();
};
