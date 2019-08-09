#include <backend/header.h>
#include "visualization/color-palette.h"
#include "visualization/color.h"
#include "visualization/common-rviz-visualization.h"

DEFINE_string(project_mat_file, "", "");
DEFINE_double(match_project_range, 20, "");
DEFINE_double(match_project_desc_diff, 50, "");
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
        
        global_matcher.MatchImg(map.frames[i], inliers_mps, inliers_kps, poses, FLAGS_match_project_range, FLAGS_match_project_desc_diff);
        frame_inliers_mps.push_back(inliers_mps);
        frame_inliers_kps.push_back(inliers_kps);
        for(int j=0; j<frame_inliers_kps[i].size(); j++){
            std::cout<<"match count: "<<i<<":"<<frame_inliers_kps[i][j].size()<<std::endl;
        }
        
    }
    
    int merge_count=0;
    int new_match_count=0;
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
                        new_match_count++;
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