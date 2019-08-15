#include <sensor_msgs/Image.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>

#include "visualization/color-palette.h"
#include "visualization/color.h"
#include "visualization/common-rviz-visualization.h"

#include "ros/ros.h"
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include "global_match/global_match.h"
#include "global_map/global_map.h"
#include "global_map/global_map_seri.h"
#include <ctime>
#include <glog/logging.h>
#include <gflags/gflags.h>
#include <tf/transform_broadcaster.h>
#include <bag_tool/extract_bag.h>
#include "global_map_api/global_map_api.h"

DEFINE_double(t_c_g_x, 0, "gps position in camera coordinate");
DEFINE_double(t_c_g_y, 0, "gps position in camera coordinate");
DEFINE_double(t_c_g_z, 0, "gps position in camera coordinate");
DEFINE_string(config_root, "", "");
DEFINE_string(map_name, "", "");
DEFINE_string(bag_addr, "", "");
DEFINE_string(res_root, "", "");
DEFINE_string(img_topic, "img", "");
DEFINE_string(gps_topic, "gps", "");

struct raw_match
{
    double timestamp;
    int gmatchnum;
    double runtime;
};

void show_mp_as_cloud(std::vector<Eigen::Vector3d>& mp_posis, std::string topic){
    Eigen::Matrix3Xd points;
    points.resize(3,mp_posis.size());
    for(int i=0; i<mp_posis.size(); i++){
        points.block<3,1>(0,i)=mp_posis[i];
    }    
    publish3DPointsAsPointCloud(points, visualization::kCommonRed, 1.0, visualization::kDefaultMapFrame,topic);
}
    
void show_pose_as_marker(std::vector<Eigen::Vector3d>& posis, std::vector<Eigen::Quaterniond>& rots, std::string topic){
    visualization::PoseVector poses_vis;
    for(int i=0; i<posis.size(); i=i+1){
        visualization::Pose pose;
        pose.G_p_B = posis[i];
        pose.G_q_B = rots[i];

        pose.id =poses_vis.size();
        pose.scale = 0.2;
        pose.line_width = 0.02;
        pose.alpha = 1;
        poses_vis.push_back(pose);
    }
    visualization::publishVerticesFromPoseVector(poses_vis, visualization::kDefaultMapFrame, "vertices", topic);
}

void Export_Raw_MatchFile(std::string& path, std::vector<raw_match>& time_matchnum_vec)
{
    std::ofstream outfile_raw;
    outfile_raw.open(path + "/raw_match.txt");
    std::vector<raw_match>::iterator time_matchnum_itor = time_matchnum_vec.begin();
    for(;time_matchnum_itor != time_matchnum_vec.end();time_matchnum_itor++)
    {
        std::stringstream raw_info;
        raw_info << std::setprecision(16) << time_matchnum_itor->timestamp << "," ;
        raw_info << time_matchnum_itor->gmatchnum << ",";
        raw_info << time_matchnum_itor->runtime << std::endl;
        outfile_raw << raw_info.str();
    }
    
    outfile_raw.close();
}
            
int main(int argc, char* argv[]){
    ros::init(argc, argv, "vis_loc");
    ros::NodeHandle nh;
    google::ParseCommandLineFlags(&argc, &argv, true);
    visualization::RVizVisualizationSink::init();
    std::string res_root=FLAGS_res_root;
    std::string map_name=FLAGS_map_name;
    std::string bag_addr=FLAGS_bag_addr;
    std::string img_topic=FLAGS_img_topic;
    
    std::vector<Eigen::Vector3d> gps_list;
    std::vector<double> gps_time_list;
    std::vector<int> gps_cov_list;
    get_gps_traj(bag_addr, FLAGS_gps_topic, gps_list, gps_time_list, gps_cov_list);
    std::vector<Eigen::Vector3d> gps_hd_list;
    for(int i=0; i<gps_list.size(); i++){
        if(gps_cov_list[i]<30){
            gps_hd_list.push_back(gps_list[i]);
        }
    }
    gm::GlobalMapApi api;
    api.init(FLAGS_config_root, FLAGS_map_name);
    api.load_map(gps_hd_list);
    std::vector<Eigen::Vector3d> mp_posis;
    api.get_pointcloud(mp_posis);
    show_mp_as_cloud(mp_posis, "global_match_test_mp");
    std::vector<Eigen::Vector3d> re_traj;
    rosbag::Bag bag;
    bag.open(bag_addr,rosbag::bagmode::Read);
    std::vector<std::string> topics;
    topics.push_back(img_topic);
    rosbag::View view(bag, rosbag::TopicQuery(topics));
    int img_count=-1;
    rosbag::View::iterator it= view.begin();
    std::vector<Eigen::Vector3d> re_posis;
    std::vector<Eigen::Quaterniond> re_quas;
    std::vector<raw_match> time_matchnum_vec;
    std::ofstream f;
    f.open(res_root+"/frame_pose_opt.txt");
    Eigen::Vector3d t_c_g;
    t_c_g<<FLAGS_t_c_g_x,FLAGS_t_c_g_y,FLAGS_t_c_g_z;
    
    for(;it!=view.end();it++){
        if(!ros::ok()){
            break;
        }
        rosbag::MessageInstance m =*it;
        sensor_msgs::CompressedImagePtr simg = m.instantiate<sensor_msgs::CompressedImage>();
        if(simg!=NULL){
            img_count++;
            if(img_count<0){
                continue;
            }
            if(img_count%5!=0){
                continue;
            }
            cv_bridge::CvImagePtr cv_ptr;
            try{
                
                cv_ptr = cv_bridge::toCvCopy(simg, "bgr8");
                cv::Mat img= cv_ptr->image;
                double timestamp = simg->header.stamp.toSec();
                cv::cvtColor(img, img, cv::COLOR_BGR2GRAY);
                std::stringstream ss_time;
                ss_time<<"img_"<<img_count<<".jpg";
                
                std::vector<int> inliers_mp;
                std::vector<cv::Point2f> inliers_kp;
                Eigen::Matrix4d pose;
                clock_t start = clock();
                Eigen::Vector3d gps_position(-1,-1,-1);
                api.locate_img(img, pose, gps_position, inliers_kp, inliers_mp);
                
                double dur = (double)(clock() - start)/CLOCKS_PER_SEC;
                raw_match su = {0};
                su.runtime = dur;
                su.timestamp = timestamp;
                su.gmatchnum = inliers_mp.size();
                time_matchnum_vec.push_back(su);
                if(inliers_mp.size()>=20){
                    re_posis.push_back(pose.block(0,3,3,1));
                    Eigen::Matrix3d rot_m=pose.block(0,0,3,3);
                    re_quas.push_back(Eigen::Quaterniond(rot_m));
                    Eigen::Vector3d gps_posi=pose.block(0,0,3,3)*t_c_g+pose.block(0,3,3,1);
                    std::stringstream ss;
                    f<<ss_time.str()<<",";

//                     f << timestamp<<","<< pose(0,0) << "," << pose(0,1)  << "," << pose(0,2) << ","  << pose(0,3) << "," <<
//                         pose(1,0) << "," << pose(1,1)  << "," << pose(1,2) << ","  << pose(1,3) << "," <<
//                         pose(2,0) << "," << pose(2,1)  << "," << pose(2,2) << ","  << pose(2,3) << std::endl;
                        f << gps_posi(0) << "," <<gps_posi(1) << "," << gps_posi(2) << std::endl;
                    
                }else{
                    f<<ss_time.str()<<std::endl;
                }
                if(img_count%1==0){
                    show_mp_as_cloud(re_posis, "global_match_test_traj");
                }
                cv::Mat debug_img;
                cv::cvtColor(img, debug_img, cv::COLOR_GRAY2RGB);
                for(int i=0; i<inliers_kp.size(); i++){
                    cv::circle(debug_img, inliers_kp[i], 4, CV_RGB(0,0,255), 2);
                }
                visualization::RVizVisualizationSink::publish("global_match_test_img", debug_img);
                if(inliers_mp.size()>=20){
                    visualization::LineSegmentVector matches;
                    for(int i=0; i<inliers_mp.size(); i++){
                        static tf::TransformBroadcaster br;
                        tf::Transform transform;
                        transform.setOrigin( tf::Vector3(re_posis.back()(0), re_posis.back()(1), re_posis.back()(2)) );
                        tf::Quaternion q(re_quas.back().x(), re_quas.back().y(), re_quas.back().z(), re_quas.back().w());
                        transform.setRotation(q);
                        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "self"));
                        //map.mappoints[inliers_mp[i]]->match_count=map.mappoints[inliers_mp[i]]->match_count+1;
                        visualization::LineSegment line_segment;
                        line_segment.from = re_posis.back();
                        line_segment.scale = 0.03;
                        line_segment.alpha = 0.2;

                        line_segment.color.red = 255;
                        line_segment.color.green = 255;
                        line_segment.color.blue = 255;
                        Eigen::Vector3d mp_posi_eig;
                        mp_posi_eig(0)=mp_posis[inliers_mp[i]].x();
                        mp_posi_eig(1)=mp_posis[inliers_mp[i]].y();
                        mp_posi_eig(2)=mp_posis[inliers_mp[i]].z();
                        line_segment.to = mp_posi_eig;
                        //std::cout<<line_segment.to.transpose()<<std::endl;
                        //std::cout<<posi_match_vec.back()<<std::endl;
                        matches.push_back(line_segment);
                    }
                    visualization::publishLines(matches, 0, visualization::kDefaultMapFrame,visualization::kDefaultNamespace, "global_match_test_line");
                }
                //std::cout<<"match count: "<<inliers_mp.size()<<std::endl;
            }catch (cv_bridge::Exception& e){
                ROS_ERROR("cv_bridge exception: %s", e.what());
                return 0;
            }
        }
    }
    f.close();
    //Export_Raw_MatchFile(res_root, time_matchnum_vec);
    //gm::save_visual_map(map, map_name);
    return 0;
}