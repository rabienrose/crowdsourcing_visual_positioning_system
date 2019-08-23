#include <backend/header.h>
#ifdef VISUALIZATION
#include "visualization/color-palette.h"
#include "visualization/color.h"
#include "visualization/common-rviz-visualization.h"
#endif

DEFINE_double(match_project_range, 20, "");
DEFINE_double(match_project_desc_diff, 50, "");
DEFINE_bool(use_se3, true, "");
void MergeMP(std::shared_ptr<gm::MapPoint> base_mp, std::shared_ptr<gm::MapPoint> to_merge_mp){
    for(int k=0; k<to_merge_mp->track.size(); k++){
        CHECK_GT(to_merge_mp->track[k].frame->obss.size(), to_merge_mp->track[k].kp_ind);
        to_merge_mp->track[k].frame->obss[to_merge_mp->track[k].kp_ind]=base_mp;
    }
}
// void show_mp_as_cloud(std::vector<Eigen::Vector3d>& mp_posis, std::string topic){
//     Eigen::Matrix3Xd points;
//     points.resize(3,mp_posis.size());
//     for(int i=0; i<mp_posis.size(); i++){
//         points.block<3,1>(0,i)=mp_posis[i];
//     }    
//     publish3DPointsAsPointCloud(points, visualization::kCommonRed, 1.0, visualization::kDefaultMapFrame,topic);
// }

void update_corresponds(gm::GlobalMap& map, std::string project_mat_file){
    chamo::GlobalMatch global_matcher;
    map.CalConnections();
    
    std::vector<std::vector<std::vector<int>>> frame_inliers_mps;
    std::vector<std::vector<std::vector<int>>> frame_inliers_kps;
    std::vector<std::vector<Eigen::Matrix4d>> posess;
    std::vector<std::shared_ptr<gm::Frame>> matchid_2_frame;
    std::vector<Eigen::Vector3d> debug_points;
    std::vector<Eigen::Vector3d> frame_points;
    global_matcher.LoadMap(project_mat_file, map, Eigen::Vector3d(-1, -1, -1));
    for(int i=0; i<map.frames.size(); i++){
        std::vector<std::vector<int>> inliers_mps;
        std::vector<std::vector<int>> inliers_kps;
        if(map.frames[i]->doMatch==false){
            continue;
        }
        std::vector<Eigen::Matrix4d> poses;
        
        global_matcher.MatchImg(map.frames[i], inliers_mps, inliers_kps, poses, FLAGS_match_project_range, FLAGS_match_project_desc_diff);
        //std::cout<<inliers_kps.size()<<std::endl;
        frame_inliers_mps.push_back(inliers_mps);
        frame_inliers_kps.push_back(inliers_kps);
        posess.push_back(poses);
        matchid_2_frame.push_back(map.frames[i]);
        for(int j=0; j<inliers_kps.size(); j++){
            if(inliers_kps.size()>=2){
                //std::cout<<"match count: "<<map.frames[i]->id<<":"<<inliers_kps[j].size()<<std::endl;
                
            }
        }
    }
    
    std::cout<<"done raw match!!"<<std::endl;
    for(int i=0; i<posess.size(); i++){
        for(int n=0; n<posess[i].size(); n++){
            CHECK_GT(frame_inliers_kps.size(), i);
            CHECK_GT(frame_inliers_kps[i].size(), n);
            if(frame_inliers_kps[i][n].size()>20){
                if((posess[i][n].block(0,3,3,1)-matchid_2_frame[i]->position).norm()>1){
                }else{
                    continue;
                }
                Eigen::Matrix4d T_tar_sour;
                double scale_tar_sour; 
                bool succ=true;
                if(FLAGS_use_se3){
                    T_tar_sour=posess[i][n]*matchid_2_frame[i]->getPose().inverse(); //to-do
                    scale_tar_sour=1;
                }else{
                    std::vector<Eigen::Vector3d> local_pc;
                    std::vector<Eigen::Vector3d> global_pc;
                    for(int j=0; j<frame_inliers_kps[i][n].size(); j++){
                        if(matchid_2_frame[i]->obss[frame_inliers_kps[i][n][j]]!=nullptr){
                            local_pc.push_back(matchid_2_frame[i]->obss[frame_inliers_kps[i][n][j]]->position);
                            global_pc.push_back(map.mappoints[frame_inliers_mps[i][n][j]]->position);
                        }
                    }
                    succ= chamo::ComputeSim3Ransac(global_pc, local_pc, T_tar_sour, scale_tar_sour);
                }
                if(succ){
                    std::map<std::shared_ptr<gm::Frame>, int> frame_list;
                    for(int j=0; j<frame_inliers_mps[i][n].size(); j++){
                        std::shared_ptr<gm::MapPoint> temp_tar_mp = map.mappoints[frame_inliers_mps[i][n][j]];
                        for(int k=0; k<temp_tar_mp->track.size(); k++){
                            if(frame_list.count(temp_tar_mp->track[k].frame)==0){
                                frame_list[temp_tar_mp->track[k].frame]=1;
                            }else{
                                frame_list[temp_tar_mp->track[k].frame]=frame_list[temp_tar_mp->track[k].frame]+1;
                            }
                        }
                    }
                    
                    std::vector<std::shared_ptr<gm::Frame> > connected_frames;
                    std::vector<int> connected_weights;
                    std::map<std::shared_ptr<gm::Frame>, int>::iterator it;
                    int max_count=0;
                    std::shared_ptr<gm::Frame> max_frame=nullptr;
                    for ( it = frame_list.begin(); it != frame_list.end(); it++ ){
                        if(it->second>30){
                            connected_weights.push_back(it->second);
                            connected_frames.push_back(it->first);
                        }
    //                         if(it->second>max_count){
    //                             max_count=it->second;
    //                             max_frame=it->first;
    //                         }
                    }
    //                     if(max_frame!= nullptr && max_count>30){
    //                         connected_frames.push_back(max_frame);
    //                     }
                    //LOG(INFO)<<"connected_frames: "<<connected_frames.size();
                    for(int j=0; j<connected_frames.size(); j++){
//                         Eigen::Matrix4d T_tarworld_sourworld = T_tar_sour;
//                         Eigen::Matrix4d T_tarworld_tar = connected_frames[j]->getPose();
//                         Eigen::Matrix4d T_sourworld_sour = matchid_2_frame[i]->getPose();
//                         Eigen::Matrix4d T_tar_tarworld = T_tarworld_tar.inverse();
//                         Eigen::Matrix4d T_tar_sour = T_tar_tarworld*T_tarworld_sourworld*T_sourworld_sour;
//                         Eigen::Matrix3d rot=T_tar_sour.block(0,0,3,3)/scale_tar_sour;
//                         Eigen::Vector3d posi=T_tar_sour.block(0,3,3,1);
                        T_tar_sour=connected_frames[j]->getPose().inverse()*posess[i][n];
                        Eigen::Matrix3d rot=T_tar_sour.block(0,0,3,3)/scale_tar_sour;
                        Eigen::Vector3d posi=T_tar_sour.block(0,3,3,1);
                        map.AddConnection(matchid_2_frame[i], connected_frames[j], posi, rot, scale_tar_sour, connected_weights[j]);
                    }
                }
            }else{
                std::cout<<"not enouph local_pc!!"<<std::endl;
                //return false;
            }
        }
    }
    int merge_count=0;
    int new_match_count=0;
    std::vector<std::set<long unsigned int >> to_merge_mps;
    for(int i=0; i<frame_inliers_kps.size(); i++){
        for(int j=0; j<frame_inliers_kps[i].size(); j++){
            if(frame_inliers_kps[i][j].size()>10){
                for(int k=0; k<frame_inliers_kps[i][j].size(); k++){
                    CHECK_GT(matchid_2_frame[i]->obss.size(), frame_inliers_kps[i][j][k]);
                    if(matchid_2_frame[i]->obss[frame_inliers_kps[i][j][k]]==nullptr){
                        CHECK_GT(map.mappoints.size(), frame_inliers_mps[i][j][k]);
                        matchid_2_frame[i]->obss[frame_inliers_kps[i][j][k]]=map.mappoints[frame_inliers_mps[i][j][k]];
                        //find new pair.
                        new_match_count++;
                    }else{
                        if(matchid_2_frame[i]->obss[frame_inliers_kps[i][j][k]]->id==map.mappoints[frame_inliers_mps[i][j][k]]->id){
                            //find the same pair, do nothing.
                        }else{
                            bool find_one=false;
                            for(int n=0; n<to_merge_mps.size(); n++){
                                auto search = to_merge_mps[n].find(matchid_2_frame[i]->obss[frame_inliers_kps[i][j][k]]->id);
                                if (search != to_merge_mps[n].end()) {
                                    to_merge_mps[n].insert(matchid_2_frame[i]->obss[frame_inliers_kps[i][j][k]]->id);
                                    to_merge_mps[n].insert(map.mappoints[frame_inliers_mps[i][j][k]]->id);
                                    find_one=true;
                                }
                                search = to_merge_mps[n].find(map.mappoints[frame_inliers_mps[i][j][k]]->id);
                                if (search != to_merge_mps[n].end()) {
                                    to_merge_mps[n].insert(matchid_2_frame[i]->obss[frame_inliers_kps[i][j][k]]->id);
                                    to_merge_mps[n].insert(map.mappoints[frame_inliers_mps[i][j][k]]->id);
                                    find_one=true;
                                }
                            }
                            if(find_one==false){
                                std::set<long unsigned int> temp_set;
                                temp_set.insert(matchid_2_frame[i]->obss[frame_inliers_kps[i][j][k]]->id);
                                temp_set.insert(map.mappoints[frame_inliers_mps[i][j][k]]->id);
                                to_merge_mps.push_back(temp_set);
                            }
                            merge_count++;
                            //find conflicted pair, merge two.
                        }
                    }
                }
            }
        }
    }
    
    std::cout<<"merge_count: "<<merge_count<<std::endl;
    std::cout<<"new_match_count: "<<new_match_count<<std::endl;
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
    int del_count=0;
    for(int i=to_dels.size()-1; i>=0; i--){
        if(to_dels[i]==true){
            del_count++;
            map.mappoints.erase(map.mappoints.begin()+i);
        }
    }
    std::cout<<"del_count: "<<del_count<<std::endl;
    map.AssignKpToMp();
}
