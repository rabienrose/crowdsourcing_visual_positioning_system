#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include<iomanip>
#include<opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include"System.h"
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/image_encodings.h>
#include <rosbag/view.h>
#include <cv_bridge/cv_bridge.h>
#include <glog/logging.h>
#include <gflags/gflags.h>

using namespace std;

DEFINE_string(bag_addr, "", "Bag contain the image data.");
DEFINE_string(output_addr, "", "Place to save the output file.");
DEFINE_string(image_topic, "img", "Topic of image in bag.");
DEFINE_int32(min_frame, 100000, "First frame to be processed.");
DEFINE_int32(max_frame, 0, "Last frame to be processed.");
DEFINE_int32(step_frame, 1, "The number of frames to be skiped.");

int main(int argc, char **argv)
{
    google::InitGoogleLogging(argv[0]);
    google::InstallFailureSignalHandler();
    google::ParseCommandLineFlags(&argc, &argv, true);
    std::string bag_str=FLAGS_bag_addr;
    std::string out_str=FLAGS_output_addr;
    std::string img_topic=FLAGS_image_topic;
    int min_frame=FLAGS_min_frame;
    int max_frame=FLAGS_max_frame;
    int step=FLAGS_step_frame;
    LOG(INFO)<<"max frame:"<<max_frame;
    std::shared_ptr<ORB_SLAM2::System> sys_p=std::make_shared<ORB_SLAM2::System>();
    rosbag::Bag bag;
    bag.open(bag_str,rosbag::bagmode::Read);
    std::vector<std::string> topics;
    topics.push_back(img_topic);
    //topics.push_back("img");
    rosbag::View view(bag, rosbag::TopicQuery(topics));
    int img_count=-1;
    rosbag::View::iterator it= view.begin();
    
    int map_count=0;
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
                bool re = sys_p->TrackMonocular(cv_ptr->image, simg->header.stamp.toSec(), ss.str());
                if(re){
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
                        //show_mp_as_cloud(posis, "vslam_output_posi");
                        cv::waitKey(1);
                    }
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
                    sys_p=std::make_shared<ORB_SLAM2::System>();
                }
                
                                
            }catch (cv_bridge::Exception& e){
                std::cout<<"err in main!!"<<std::endl;
                return 0;
            }
        }
    }
    std::stringstream ss;
    ss<<map_count;
    out_str=out_str+"/"+"submap_"+ss.str()+".map";
    sys_p->saveToVisualMap(out_str);
    
    return 0;
}
