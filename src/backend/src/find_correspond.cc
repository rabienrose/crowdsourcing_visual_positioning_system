#include <backend/header.h>
#include "visualization/color-palette.h"
#include "visualization/color.h"
#include "visualization/common-rviz-visualization.h"

DEFINE_string(project_mat_file, "", "");

void MergeMP(std::shared_ptr<gm::MapPoint> base_mp, std::shared_ptr<gm::MapPoint> to_merge_mp){
    for(int k=0; k<to_merge_mp->track.size(); k++){
        CHECK_GT(to_merge_mp->track[k].frame->obss.size(), to_merge_mp->track[k].kp_ind);
        to_merge_mp->track[k].frame->obss[to_merge_mp->track[k].kp_ind]=base_mp;
    }
}
void show_mp_as_cloud(std::vector<Eigen::Vector3d>& mp_posis, std::string topic){
    Eigen::Matrix3Xd points;
    points.resize(3,mp_posis.size());
    for(int i=0; i<mp_posis.size(); i++){
        points.block<3,1>(0,i)=mp_posis[i];
    }    
    publish3DPointsAsPointCloud(points, visualization::kCommonRed, 1.0, visualization::kDefaultMapFrame,topic);
}

void update_corresponds(gm::GlobalMap& map){
    chamo::GlobalMatch global_matcher;
    global_matcher.LoadMap(FLAGS_project_mat_file, map, map.frames[0]->position);
    
    std::vector<std::vector<std::vector<int>>> frame_inliers_mps;
    std::vector<std::vector<std::vector<int>>> frame_inliers_kps;
    std::vector<Eigen::Vector3d> debug_points;
    for(int i=0; i<map.frames.size(); i++){
        std::vector<std::vector<int>> inliers_mps;
        std::vector<std::vector<int>> inliers_kps;
        if(map.frames[i]->doMatch==false){
            frame_inliers_mps.push_back(inliers_mps);
            frame_inliers_kps.push_back(inliers_kps);
            continue;
        }
        std::vector<Eigen::Matrix4d> poses;
        global_matcher.MatchImg(map.frames[i], inliers_mps, inliers_kps, poses);
        frame_inliers_mps.push_back(inliers_mps);
        frame_inliers_kps.push_back(inliers_kps);
        std::cout<<"frame: "<<poses.size()<<std::endl;
        for(int n=0; n<poses.size(); n++){
            if(inliers_kps[n].size()>30){
                if((poses[n].block(0,3,3,1)-map.frames[i]->position).norm()>3){
                    map.frames[i]->doGraphOpti=true;
                    //LOG(INFO)<<"global match count: "<<inliers_kps[n].size();
                    //std::cout<<"do graph"<<std::endl;
                }else{
                    map.frames[i]->doGraphOpti=true;
                    //std::cout<<"not do graph"<<std::endl;
                    continue;
                }
                std::vector<Eigen::Vector3d> local_pc;
                std::vector<Eigen::Vector3d> global_pc;
                for(int j=0; j<inliers_kps[n].size(); j++){
                    if(map.frames[i]->obss[inliers_kps[n][j]]!=nullptr){
                        local_pc.push_back(map.frames[i]->obss[inliers_kps[n][j]]->position);
                        global_pc.push_back(map.mappoints[inliers_mps[n][j]]->position);
                    }
                }
                Eigen::Matrix4d T_tar_sour;
                double scale_tar_sour; 
                //show_mp_as_cloud(global_pc, "global_pc");
                //show_mp_as_cloud(local_pc, "local_pc");
                //LOG(INFO)<<"global_pc: "<<global_pc.size();
               // LOG(INFO)<<"local_pc: "<<local_pc.size();
                bool succ= chamo::ComputeSim3Ransac(global_pc, local_pc, T_tar_sour, scale_tar_sour);
                if(succ){
                    //LOG(INFO)<<"scale_tar_sour: "<<scale_tar_sour;
                    std::map<std::shared_ptr<gm::Frame>, int> frame_list;
                    for(int j=0; j<inliers_mps[n].size(); j++){
                        std::shared_ptr<gm::MapPoint> temp_tar_mp = map.mappoints[inliers_mps[n][j]];
                        for(int k=0; k<temp_tar_mp->track.size(); k++){
                            if(frame_list.count(temp_tar_mp->track[k].frame)==0){
                                frame_list[temp_tar_mp->track[k].frame]=1;
                            }else{
                                frame_list[temp_tar_mp->track[k].frame]=frame_list[temp_tar_mp->track[k].frame]+1;
                            }
                        }
                    }
                    
                    std::vector<std::shared_ptr<gm::Frame> > connected_frames;
                    std::map<std::shared_ptr<gm::Frame>, int>::iterator it;
                    int max_count=0;
                    std::shared_ptr<gm::Frame> max_frame=nullptr;
                    for ( it = frame_list.begin(); it != frame_list.end(); it++ ){
                        if(it->second>30){
                            connected_frames.push_back(it->first);
                        }
                        if(it->second>max_count){
                            max_count=it->second;
                            max_frame=it->first;
                        }
                    }
//                     if(max_frame!= nullptr && max_count>30){
//                         connected_frames.push_back(max_frame);
//                     }
                    //LOG(INFO)<<"connected_frames: "<<connected_frames.size();
                    for(int j=0; j<connected_frames.size(); j++){
                        Eigen::Matrix4d T_tarworld_sourworld = T_tar_sour;
                        Eigen::Matrix4d T_tarworld_tar = connected_frames[j]->getPose();
                        Eigen::Matrix4d T_sourworld_sour = map.frames[i]->getPose();
                        Eigen::Matrix4d T_tar_tarworld = T_tarworld_tar.inverse();
                        Eigen::Matrix4d T_tar_sour = T_tar_tarworld*T_tarworld_sourworld*T_sourworld_sour;
                        Eigen::Matrix3d rot=T_tar_sour.block(0,0,3,3)/scale_tar_sour;
                        Eigen::Vector3d posi=T_tar_sour.block(0,3,3,1);
                        map.AddConnection(map.frames[i], connected_frames[j], posi, rot, scale_tar_sour, max_count);

                    }
    //                 Eigen::Matrix4d T_sourworld_sour = source_map.frames[i]->getPose();
    //                 Eigen::Vector4d posi_homo;
    //                 posi_homo.block(0,0,3,1)=T_sourworld_sour.block(0,3,3,1);
    //                 posi_homo(3)=1;
    //                 Eigen::Vector4d sim_posi = T_tar_sour*posi_homo;
                    //LOG(INFO)<<"sim_posi: "<<sim_posi.block(0,0,3,1).transpose();
                }else{
                    //std::cout<<"ransac for local pc match failed!!"<<std::endl;
                    //return false;
                }
                
            }else{
                std::cout<<"not enouph local_pc!!"<<std::endl;
                //return false;
            }
        }
        //
    }
    std::vector<std::set<long unsigned int >> to_merge_mps;
    for(int i=0; i<frame_inliers_kps.size(); i++){
        for(int j=0; j<frame_inliers_kps[i].size(); j++){
            if(frame_inliers_kps[i][j].size()>10){
                for(int k=0; k<frame_inliers_kps[i][j].size(); k++){
                    CHECK_GT(map.frames[i]->obss.size(), frame_inliers_kps[i][j][k]);
                    if(map.frames[i]->obss[frame_inliers_kps[i][j][k]]==nullptr){
                        CHECK_GT(map.mappoints.size(), frame_inliers_mps[i][j][k]);
                        map.frames[i]->obss[frame_inliers_kps[i][j][k]]=map.mappoints[frame_inliers_mps[i][j][k]];
                        //find new pair.
                    }else{
                        if(map.frames[i]->obss[frame_inliers_kps[i][j][k]]->id==map.mappoints[frame_inliers_mps[i][j][k]]->id){
                            //find the same pair, do nothing.
                        }else{
                            bool find_one=false;
                            for(int n=0; n<to_merge_mps.size(); n++){
                                auto search = to_merge_mps[n].find(map.frames[i]->obss[frame_inliers_kps[i][j][k]]->id);
                                if (search != to_merge_mps[n].end()) {
                                    to_merge_mps[n].insert(map.frames[i]->obss[frame_inliers_kps[i][j][k]]->id);
                                    to_merge_mps[n].insert(map.mappoints[frame_inliers_mps[i][j][k]]->id);
                                    find_one=true;
                                }
                                search = to_merge_mps[n].find(map.mappoints[frame_inliers_mps[i][j][k]]->id);
                                if (search != to_merge_mps[n].end()) {
                                    to_merge_mps[n].insert(map.frames[i]->obss[frame_inliers_kps[i][j][k]]->id);
                                    to_merge_mps[n].insert(map.mappoints[frame_inliers_mps[i][j][k]]->id);
                                    find_one=true;
                                }
                            }
                            if(find_one==false){
                                std::set<long unsigned int> temp_set;
                                temp_set.insert(map.frames[i]->obss[frame_inliers_kps[i][j][k]]->id);
                                temp_set.insert(map.mappoints[frame_inliers_mps[i][j][k]]->id);
                                to_merge_mps.push_back(temp_set);
                            }
                            //find conflicted pair, merge two.
                        }
                    }
                }
            }
        }
    }
    std::map<long unsigned int, int> mpid_to_vecter_id;
    std::vector<bool> to_dels;
    for(int i=0; i<map.mappoints.size(); i++){
        mpid_to_vecter_id[map.mappoints[i]->id]=i;
        to_dels.push_back(false);
    }
    for(int i=0; i<to_merge_mps.size(); i++){
        //std::cout<<"merged count: "<<to_merge_mps[i].size()<<std::endl;
        std::set<long unsigned int>::iterator it;
        std::shared_ptr<gm::MapPoint> first_mp=nullptr;
        for (it = to_merge_mps[i].begin(); it != to_merge_mps[i].end(); ++it){
            if(first_mp!=nullptr){
                MergeMP(first_mp, map.mappoints[mpid_to_vecter_id[*it]]);
                to_dels[mpid_to_vecter_id[*it]]=true;
            }else{
                first_mp=map.mappoints[mpid_to_vecter_id[*it]];
            }
        }
    }

    for(int i=to_dels.size()-1; i>=0; i--){
        if(to_dels[i]==true){
            map.mappoints.erase(map.mappoints.begin()+i);
        }
        
    }
    int pose_opt_count=0;
    for(int n=0; n<map.frames.size(); n++){
        if(map.frames[n]->doGraphOpti==false){
            continue;
        }
        std::vector<std::shared_ptr<gm::Frame>> cur_level_frames;
        cur_level_frames.push_back(map.frames[n]);
        for(int i=0; i<10; i++){
            std::vector<std::shared_ptr<gm::Frame>> next_level_frames;
            for(int j=0; j<cur_level_frames.size(); j++){
                
                std::vector<std::shared_ptr<gm::Frame>> next_level_frames_temp;
                map.getFrameChildren(next_level_frames_temp, cur_level_frames[j]);
                for(int k=0; k<next_level_frames_temp.size(); k++){
                    if(next_level_frames_temp[k]->doGraphOpti==false){
                        next_level_frames_temp[k]->doGraphOpti=true;
                        pose_opt_count++;
                        next_level_frames.push_back(next_level_frames_temp[k]);
                    }
                }
            }
            cur_level_frames=next_level_frames;
        }
        for(int j=0; j<cur_level_frames.size(); j++){
            cur_level_frames[j]->doGraphOpti=true;
            cur_level_frames[j]->isfix=true;
        }
    }
    std::cout<<"pose edge count after match: "<<map.pose_graph_v1.size()<<std::endl;
    std::cout<<"pose opt count: "<<pose_opt_count<<std::endl;
    
    map.AssignKpToMp();
}