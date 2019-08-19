#include <string>
#include <fstream>
#include <memory>
#include <Eigen/Core>
#include <math.h>
#include <unordered_map>

#include <backend/header.h>
#include "opencv2/opencv.hpp"
#include <glog/logging.h>
#include <gflags/gflags.h>
#include "chamo_common/common.h"
#include <iomanip>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "System.h"
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/image_encodings.h>
#include <rosbag/view.h>
#include <cv_bridge/cv_bridge.h>
#include "global_map/global_map.h"
#include "global_map/global_map_seri.h"
#include "ORBextractor.h"
#include "global_map_api/global_map_api.h"
#include "bag_tool/extract_bag.h"
#include "convert_to_visual_map/convert_to_visual_map.h"
#include "backend/header.h"
#include "merge_new/merge_new.h"
#include "global_map_api/global_map_api.h"

#ifdef VISUALIZATION
#include "visualization/color-palette.h"
#include "visualization/color.h"
#include "visualization/common-rviz-visualization.h"
void show_mp_as_cloud(std::vector<Eigen::Vector3d>& mp_posis, std::string topic){
    Eigen::Matrix3Xd points;
    points.resize(3,mp_posis.size());
    for(int i=0; i<mp_posis.size(); i++){
        points.block<3,1>(0,i)=mp_posis[i];
    }    
    publish3DPointsAsPointCloud(points, visualization::kCommonRed, 1.0, visualization::kDefaultMapFrame,topic);
}
#endif
DECLARE_string(voc_addr);
DECLARE_string(camera_config);
DECLARE_double(max_repro_err);
namespace gm{
    
    void do_vslam(std::string local_addr, std::string config_addr, std::string bag_addr){
        FLAGS_voc_addr=config_addr+"/FreakAll.bin";
        FLAGS_camera_config=config_addr+"/camera_config.txt";
        std::string out_str=local_addr;
        std::string img_topic="img";
        int min_frame=2;
        int max_frame=10000;
        int step=1;
        LOG(INFO)<<"max frame:"<<max_frame;
        ORB_SLAM2::System* sys_p=nullptr;
        rosbag::Bag bag;
        bag.open(bag_addr,rosbag::bagmode::Read);
        std::vector<std::string> topics;
        topics.push_back(img_topic);
        //topics.push_back("img");
        rosbag::View view(bag, rosbag::TopicQuery(topics));
        int img_count=-1;
        rosbag::View::iterator it= view.begin();
        
        int map_count=0;
        bool bReset=false;
        for(;it!=view.end();it++){
            rosbag::MessageInstance m =*it;
            sensor_msgs::CompressedImagePtr simg = m.instantiate<sensor_msgs::CompressedImage>();
            if(simg!=NULL){
                cv_bridge::CvImagePtr cv_ptr;
                img_count++;
                if(img_count%step!=0){
                    continue;
                }
                if(img_count <min_frame){
                    continue;
                }
                if(img_count >max_frame){
                    break;
                }
                try{
                    std::stringstream ss;
                    ss<<"img_"<<img_count<<".jpg";
                    cv_ptr = cv_bridge::toCvCopy(simg, "mono8");
                    cv::Mat resize_img;
                    //cv::resize(cv_ptr->image, resize_img, cv::Size(cv_ptr->image.cols/2, cv_ptr->image.rows/2));
                    if(cv_ptr->image.empty()){
                        std::cout<<"image empty"<<std::endl;
                    }
                    if(sys_p==nullptr){
                        sys_p=new ORB_SLAM2::System();
                    }
                    bool re = sys_p->TrackMonocular(cv_ptr->image, simg->header.stamp.toSec(), ss.str());
                   std::cout<<img_count<<std::endl;
//                    if(img_count%300==0){
//                        re=false;
//                    }else{
//                        re=true;
//                    }
                    if(re){
#ifdef VISUALIZATION
                        std::vector<Eigen::Vector3d> pcs;
                        sys_p->getPC(pcs);
                        std::vector<Eigen::Vector3d> posis;
                        std::vector<Eigen::Quaterniond> quas;
                        sys_p->getTraj(posis, quas);
                        float reproject_err_t;
                        int match_count_t;
                        int mp_count_t;
                        int kf_count_t;
                        cv::Mat img_display;
                        sys_p->getDebugImg(img_display, reproject_err_t, match_count_t, mp_count_t, kf_count_t);

                        if(!img_display.empty()){
                            cv::imshow("chamo", img_display);
                            show_mp_as_cloud(posis, "vslam_output_posi");
                            cv::waitKey(1);
                        }
#endif
                    }else{
                        std::vector<Eigen::Vector3d> posis;
                        std::vector<Eigen::Quaterniond> quas;
                        sys_p->getTraj(posis, quas);
                        if(posis.size()>10){
                            std::stringstream ss;
                            ss<<map_count;
                            sys_p->saveToVisualMap(out_str+"/"+"submap_"+ss.str()+".map");
                            map_count++;
                        }
                        if(sys_p!=nullptr){
                            delete sys_p;
                            sys_p=nullptr;
                        }
                    }
                }catch (cv_bridge::Exception& e){
                    std::cout<<"err in main!!"<<std::endl;
                    if(sys_p!=nullptr){
                        delete sys_p;
                        sys_p=nullptr;
                    }
                    return;
                }
            }
        }
        std::stringstream ss;
        ss<<map_count;
        out_str=out_str+"/"+"submap_"+ss.str()+".map";
        sys_p->saveToVisualMap(out_str);
        if(sys_p!=nullptr){
            delete sys_p;
            sys_p=nullptr;
        }
    }
    
    void read_cam_info(std::string cam_addr, Eigen::Matrix3d& cam_inter, Eigen::Vector4d& cam_distort){
        std::string line;
        std::ifstream infile_camera(cam_addr.c_str()); 
        std::getline(infile_camera, line);
        std::vector<std::string> splited = chamo::split(line, ",");
        cam_inter=Eigen::Matrix3d::Identity();
        cam_inter(0,0)=atof(splited[0].c_str());
        cam_inter(1,1)=atof(splited[1].c_str());
        cam_inter(0,2)=atof(splited[2].c_str());
        cam_inter(1,2)=atof(splited[3].c_str());
        cam_distort(0)=atof(splited[4].c_str());
        cam_distort(1)=atof(splited[5].c_str());
        cam_distort(2)=atof(splited[6].c_str());
        cam_distort(3)=atof(splited[7].c_str());
    }

    bool GlobalMapApi::init(std::string config_addr_, std::string map_addr_){
        map_addr=map_addr_;
        config_addr=config_addr_;
        cur_map=nullptr;
        Eigen::Matrix3d cam_inter;
        Eigen::Vector4d cam_distort;
        read_cam_info(config_addr+"/camera_config.txt", cam_inter, cam_distort);
        fx=cam_inter(0,0);
        fy=cam_inter(1,1);
        cx=cam_inter(0,2);
        cy=cam_inter(1,2);
        k1=cam_distort(0);
        k2=cam_distort(1);
        p1=cam_distort(2);
        p2=cam_distort(3);
        return true;
    }
    bool GlobalMapApi::load_map(std::vector<Eigen::Vector3d> gps_positions){
        cur_map=std::make_shared<GlobalMap>();
        matcher=std::make_shared<chamo::GlobalMatch>();
        extractor=std::make_shared<ORB_SLAM2::ORBextractor>(2000, 1.2, 8, 20, 7);
        GlobalMap temp_map;
        std::vector<unsigned int> map_ids;
        gm::get_blockids_frome_gps_list(gps_positions, map_ids);
        gm::load_global_map(temp_map, map_addr,map_ids);
        temp_map.AssignKpToMp();
        *cur_map=temp_map;
        Eigen::Vector3d est_posi(-1, -1,-1);
        matcher->LoadMap(config_addr+"/words_projmat.fstream", *cur_map ,est_posi);
        return true;
    }
    bool GlobalMapApi::get_pointcloud(std::vector<Eigen::Vector3d>& out_pointcloud, std::vector<unsigned int> ids){
        gm::fast_load_mps(out_pointcloud, map_addr, ids);
        return true;
    }
    
    bool GlobalMapApi::process_bag(std::string bag_addr, std::string cache_addr, std::string localmap_addr){
        //extract_bag(cache_addr, bag_addr, "img", "imu", "gps", false);
        //do_vslam(cache_addr, config_addr, bag_addr);
        std::vector<unsigned int> block_ids;
        block_ids.push_back(112224160);
        convert_to_visual_map(config_addr, cache_addr,localmap_addr, block_ids);
        merge_new(map_addr, localmap_addr, map_addr, block_ids);
        gm::GlobalMap map;
        gm::load_global_map(map, map_addr,block_ids);
        map.AssignKpToMp();
        update_corresponds(map, config_addr+"/words_projmat.fstream");
        //pose_graph_opti_se3(map);
//        FLAGS_max_repro_err=100;
//        optimize_BA(map, false);
//        FLAGS_max_repro_err=20;
//        optimize_BA(map, false);
//        culling_frame(map);
//        reset_all_status(map, "all", false);
//        gm::save_global_map(map, map_addr);
        return true;
    }
    bool GlobalMapApi::locate_img(cv::Mat img, Eigen::Matrix4d& pose, Eigen::Vector3d gps_position, 
                                  std::vector<cv::Point2f>& inliers_kp, std::vector<int>& inliers_mp){
        std::vector<Eigen::Matrix4d> poses;
        
        cv::Mat descriptors;
        std::vector<cv::KeyPoint> keypoints;
        cv::Mat undistort_img;
        cv::Mat K = cv::Mat::eye(3,3,CV_32F);
        K.at<float>(0,0) = fx;
        K.at<float>(1,1) = fy;
        K.at<float>(0,2) = cx;
        K.at<float>(1,2) = cy;

        cv::Mat DistCoef(4,1,CV_32F);
        DistCoef.at<float>(0) = k1;
        DistCoef.at<float>(1) = k2;
        DistCoef.at<float>(2) = p1;
        DistCoef.at<float>(3) = p2;
        cv::undistort(img, undistort_img, K, DistCoef);
        
        extractor->ExtractDesc(undistort_img, cv::Mat() ,keypoints, descriptors, false);
        
        std::shared_ptr<Frame> loc_frame=std::make_shared<Frame>();
        loc_frame->fx=fx;
        loc_frame->fy=fy;
        loc_frame->cx=cx;
        loc_frame->cy=cy;
        loc_frame->k1=0;
        loc_frame->k2=0;
        loc_frame->p1=0;
        loc_frame->p2=0;
        loc_frame->kps= keypoints;
        int desc_width=descriptors.cols;
        int desc_count=descriptors.rows;
        loc_frame->descriptors.resize(desc_width, desc_count);
        for(int j=0; j<desc_width; j++){
            for(int k=0; k<desc_count; k++){
                loc_frame->descriptors(j, k)=descriptors.at<unsigned char>(k, j);
            } 
        }
        std::vector<std::vector<int>> inliers_kps;
        std::vector<std::vector<int>> inliers_mps;
        matcher->MatchImg(loc_frame, inliers_mps, inliers_kps, poses, 20, 50);
        if(poses.size()>0){
            pose=poses[0];
            for(int i=0; i<inliers_kps[0].size(); i++){
                inliers_kp.push_back(loc_frame->kps[inliers_kps[0][i]].pt);
            }
            inliers_mp=inliers_mps[0];
            return true;
        }else{
            return false;
        }
    }        
}



