#include "visualization/color-palette.h"
#include "visualization/color.h"
#include "visualization/common-rviz-visualization.h"
#include "global_map/global_map.h"
#include "global_map/global_map_seri.h"

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
          
int main(int argc, char* argv[]){
    
    google::InitGoogleLogging(argv[0]);
    google::InstallFailureSignalHandler();
    google::ParseCommandLineFlags(&argc, &argv, true);
    visualization::RVizVisualizationSink::init();
    std::string res_root=argv[1];
    int map_count=argc-2;
    std::vector<unsigned int> map_ids;
    for(int i=2; i<argc; i++){
        unsigned int map_id=std::stoul(argv[i]);
        map_ids.push_back(map_id);
    }
    gm::GlobalMap map;
    //gm::load_submap(map, res_root,true);
    gm::load_global_map(map, res_root,map_ids);
    //map.AssignKpToMp();
    std::vector<Eigen::Vector3d> traj_posi;
    std::vector<Eigen::Vector3d> mp_posi;
    std::vector<Eigen::Vector3d> gps_posi;
    std::vector<Eigen::Vector3d> hd_gps_posi;
    for(int i=0; i<map.frames.size(); i++){
        traj_posi.push_back(map.frames[i]->position);
        //std::cout<<map.frames[i]->position.transpose()<<std::endl;
    }
    for(int i=0; i<map.frames.size(); i++){
        gps_posi.push_back(map.frames[i]->gps_position);
    }
    for(int i=0; i<map.frames.size(); i++){
        if(map.frames[i]->gps_accu<5){
            hd_gps_posi.push_back(map.frames[i]->gps_position);
        }
        
    }
    for(int i=0; i<map.mappoints.size(); i++){
        mp_posi.push_back(map.mappoints[i]->position);
    }
    std::vector<Eigen::Quaterniond> rots;
    for(int i=0; i<map.frames.size(); i++){
        rots.push_back(map.frames[i]->direction);
    }
    show_pose_as_marker(traj_posi, rots, "vm_pose");
    visualization::LineSegmentVector matches;
    for(int i=0; i<map.pose_graph_e_posi.size(); i++){
        visualization::LineSegment line_segment;
        line_segment.from = map.pose_graph_v1[i]->position;
        line_segment.scale = 0.003;
        line_segment.alpha = 1;

        line_segment.color.red = 0;
        line_segment.color.green = 255;
        line_segment.color.blue = 0;
        line_segment.to = map.pose_graph_v2[i]->position;
        matches.push_back(line_segment);
    }
    visualization::publishLines(matches, 0, visualization::kDefaultMapFrame,visualization::kDefaultNamespace, "vm_covisibility");
    show_mp_as_cloud(traj_posi, "vm_frame_posi");
    show_mp_as_cloud(mp_posi, "vm_mp_posi");
    show_mp_as_cloud(gps_posi, "vm_gps_posi");
    show_mp_as_cloud(hd_gps_posi, "vm_hd_gps_posi");
    ros::Rate loop_rate(10);
    for(int i=0; i<map.frames.size(); i++){
        visualization::LineSegmentVector matches;
        double t_proj_err=0;
        int mp_count=0;
        for(int j=0; j<map.frames[i]->obss.size(); j++){
            if(map.frames[i]->obss[j]!=nullptr){
                visualization::LineSegment line_segment;
                line_segment.from = map.frames[i]->position;
                line_segment.scale = 0.3;
                line_segment.alpha = 0.6;

                line_segment.color.red = 255;
                line_segment.color.green = 255;
                line_segment.color.blue = 255;
                line_segment.to = map.frames[i]->obss[j]->position;
                matches.push_back(line_segment);
                
                Eigen::Matrix<double, 3,4> proj_mat = map.frames[i]->getProjMat();
                Eigen::Vector4d posi_homo;
                posi_homo.block(0,0,3,1)=map.frames[i]->obss[j]->position;
                posi_homo(3)=1;
                Eigen::Vector3d proj_homo = proj_mat*posi_homo;
                double u=proj_homo(0)/proj_homo(2);
                double v=proj_homo(1)/proj_homo(2);
                cv::Point2f uv= map.frames[i]->kps[j].pt;
                //std::cout<<u<<":"<<v<<"     "<<uv.x<<":"<<uv.y<<std::endl;
                
                float proj_err=sqrt((uv.x-u)*(uv.x-u)+(uv.y-v)*(uv.y-v));
                if(proj_err>10000 || proj_err<-10000){
                    std::cout<<uv.x<<":"<<uv.x<<":"<<v<<":"<<u<<std::endl;
                    std::cout<<map.frames[i]->getPose()<<std::endl;
                }
                
                t_proj_err=t_proj_err+proj_err;
                mp_count++;
            }
        }
        std::cout<<"avg. proj. err.: "<<t_proj_err<<":"<<mp_count<<std::endl;
        visualization::publishLines(matches, 0, visualization::kDefaultMapFrame,visualization::kDefaultNamespace, "vm_matches");
        if(ros::ok()){
            loop_rate.sleep();
        }else{
            break;
        }
    }
    ros::spin();

    return 0;
}