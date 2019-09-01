#include <backend/header.h>
#include <chamo_common/common.h>
#include <sim3_ransac/sim3_match.h>

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
    
    bool do_sim3_trans=true;
    if(do_sim3_trans){
        std::vector<std::vector<std::shared_ptr<gm::Frame>>> group_frames;
        cal_subgroup(map, group_frames);
        for(int i=0; i<group_frames.size(); i++){
            std::cout<<group_frames[i].size()<<std::endl;
        }
        std::map<int, int> groupid_to_sizes;
        std::map<std::shared_ptr<gm::Frame>, int> frame_to_groupid;
        for(int i=0; i<group_frames.size(); i++){
            groupid_to_sizes[i]=group_frames[i].size();
            std::cout<<i<<"::"<<groupid_to_sizes[i]<<std::endl;
            for(int j=0; j<group_frames[i].size(); j++){
                frame_to_groupid[group_frames[i][j]]=i;
            }
        }
        typedef std::function<bool(std::pair<int, int>, std::pair<int, int>)> Comparator;
        Comparator compFunctor =
                [](std::pair<int, int> elem1 ,std::pair<int, int> elem2)
                {
                    return elem1.second > elem2.second;
                };
        std::set<std::pair<int, int>, Comparator> setOfWords(
                groupid_to_sizes.begin(), groupid_to_sizes.end(), compFunctor);
        std::vector<int> ranked_groupid;
        for (std::pair<int, int> element : setOfWords){
            ranked_groupid.push_back(element.first);
            
        }
        for(int i=0; i<ranked_groupid.size(); i++){
            std::cout<<"sdfsdf: "<<group_frames[ranked_groupid[i]].size()<<std::endl;
        }
        std::vector<int> groupid_1;
        std::vector<int> groupid_2;
        std::vector<Eigen::Vector3d> posi_1;
        std::vector<Eigen::Vector3d> posi_2;
        for(int i=0; i<frame_inliers_mps.size(); i++){
            for(int j=0; j<frame_inliers_mps[i].size(); j++){
                for(int k=0; k<frame_inliers_mps[i][j].size(); k++){
                    std::shared_ptr<gm::MapPoint> mp1_t = matchid_2_frame[i]->obss[frame_inliers_kps[i][j][k]];
                    if(mp1_t!=nullptr){
                        if(mp1_t->track.size()==0){
                            continue;
                        }
                        int group1;
                        Eigen::Vector3d posi1;
                        auto it1=frame_to_groupid.find(mp1_t->track[0].frame);
                        if(it1==frame_to_groupid.end()){
                            continue;
                        }
                        group1=it1->second;
                        posi1=mp1_t->position;
                        std::shared_ptr<gm::MapPoint> mp2_t=map.mappoints[frame_inliers_mps[i][j][k]];
                        for(int n=0; n<mp2_t->track.size(); n++){
                            int group2;
                            Eigen::Vector3d posi2;
                            auto it2=frame_to_groupid.find(mp2_t->track[n].frame);
                            if(it2==frame_to_groupid.end()){
                                continue;
                            }
                            group2=it2->second;
                            if(group2==group1){
                                continue;
                            }
                            posi2=mp2_t->position;
                            groupid_1.push_back(group1);
                            groupid_2.push_back(group2);
                            posi_1.push_back(posi1);
                            posi_2.push_back(posi2);
                        }
                    }
                }
            }
        }
        std::vector<int> group_sim3_1;
        std::vector<int> group_sim3_2;
        std::vector<Eigen::Matrix4d> group_sim3;
        for(int i=0; i<group_frames.size()-1; i++){
            for(int j=i+1; j<group_frames.size(); j++){
                std::vector<Eigen::Vector3d> pc1;
                std::vector<Eigen::Vector3d> pc2;
                for(int k=0; k<groupid_1.size(); k++){
                    if(groupid_1[k]==i && groupid_2[k]==j){
                        pc1.push_back(posi_1[k]);
                        pc2.push_back(posi_2[k]);
                    }
                    if(groupid_1[k]==j && groupid_2[k]==i){
                        pc1.push_back(posi_2[k]);
                        pc2.push_back(posi_1[k]);
                    }
                }
                if(pc1.size()<200){
                    continue;
                }
                Eigen::Matrix4d T12;
                double scale_12;
                bool re = chamo::ComputeSim3Ransac(pc1, pc2, T12, scale_12);
                std::cout<<i<<":"<<j<<std::endl;
                std::cout<<pc1.size()<<":"<<pc2.size()<<std::endl;
                std::cout<<T12<<std::endl;
                std::cout<<scale_12<<std::endl;
                if(re){
                    std::cout<<"succ"<<std::endl;
                    group_sim3_1.push_back(i);
                    group_sim3_2.push_back(j);
                    group_sim3.push_back(T12);
                }
            }
        }
        
        for(int i=1; i<ranked_groupid.size(); i++){ //s
            for(int k=0; k<i; k++){ //l
                int trans_ind=-1;
                Eigen::Matrix4d T_1_t;
                for(int j=0; j<group_sim3_1.size(); j++){
                    if(group_sim3_1[j]==ranked_groupid[i] && group_sim3_2[j]==ranked_groupid[k]){
                        trans_ind=j;
                        T_1_t = group_sim3[trans_ind].inverse(); //group_sim3 is from 2 to 1, but 1 is the smaller one, so should convert 1 to 2
                        std::cout<<"1 is smaller"<<std::endl;
                        break;
                    }
                }
                for(int j=0; j<group_sim3_1.size(); j++){
                    if(group_sim3_1[j]==ranked_groupid[k] && group_sim3_2[j]==ranked_groupid[i]){
                        trans_ind=j;
                        T_1_t = group_sim3[trans_ind]; //group_sim3 is from 2 to 1, 2 is the smaller one
                        std::cout<<"2 is smaller"<<std::endl;
                        break;
                    }
                }
                if(trans_ind==-1){
                    continue;
                }
                
                for(int n=0; n<group_frames[ranked_groupid[i]].size(); n++){
                    Eigen::Matrix4d pose_transformed_temp;
                    Eigen::Matrix4d temp_pose=group_frames[ranked_groupid[i]][n]->getPose();
                    chamo::transformPoseUseSim3(T_1_t, temp_pose, pose_transformed_temp);
                    group_frames[ranked_groupid[i]][n]->setPose(pose_transformed_temp);
                }
                std::set<std::shared_ptr<gm::MapPoint>> need_mod_mps;
                for(int n=0; n<group_frames[ranked_groupid[i]].size(); n++){
                    for(int m=0; m<group_frames[ranked_groupid[i]][n]->obss.size(); m++){
                        if(group_frames[ranked_groupid[i]][n]->obss[m]!=nullptr){
                            need_mod_mps.insert(group_frames[ranked_groupid[i]][n]->obss[m]);
                        }
                    }
                }
                
                for(auto mp : need_mod_mps){
                    Eigen::Vector4d posi_homo;
                    posi_homo.block(0,0,3,1)=mp->position;
                    posi_homo(3)=1;
                    Eigen::Vector4d posi_gps_homo = T_1_t*posi_homo;
                    mp->position=posi_gps_homo.block(0,0,3,1);   
                }
                break;
            }
        }
        frame_inliers_mps.clear();
        frame_inliers_kps.clear();
        posess.clear();
        matchid_2_frame.clear();
        for(int i=0; i<map.frames.size(); i++){
            std::vector<std::vector<int>> inliers_mps;
            std::vector<std::vector<int>> inliers_kps;
            if(map.frames[i]->doMatch==false){
                continue;
            }
            std::vector<Eigen::Matrix4d> poses;
            global_matcher.MatchImg(map.frames[i], inliers_mps, inliers_kps, poses, FLAGS_match_project_range, FLAGS_match_project_desc_diff);
            frame_inliers_mps.push_back(inliers_mps);
            frame_inliers_kps.push_back(inliers_kps);
            posess.push_back(poses);
            matchid_2_frame.push_back(map.frames[i]);
        }
    }
    
    map.AssignKpToMp();
    for(int i=0; i<posess.size(); i++){
        for(int n=0; n<posess[i].size(); n++){
            CHECK_GT(frame_inliers_kps.size(), i);
            CHECK_GT(frame_inliers_kps[i].size(), n);
            if(frame_inliers_kps[i][n].size()>20){
//                 if((posess[i][n].block(0,3,3,1)-matchid_2_frame[i]->position).norm()>1){
//                 }else{
//                     continue;
//                 }
                Eigen::Matrix4d T_tar_sour;
                double scale_tar_sour; 
                bool succ=true;
                if(FLAGS_use_se3){
                    T_tar_sour=posess[i][n]*matchid_2_frame[i]->getPose().inverse();
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
                    //std::cout<<"=========================="<<std::endl;
                    //std::cout<<"matched count: "<<frame_inliers_mps[i][n].size()<<std::endl;
                    for(int j=0; j<frame_inliers_mps[i][n].size(); j++){
                        std::shared_ptr<gm::MapPoint> temp_tar_mp = map.mappoints[frame_inliers_mps[i][n][j]];
                        for(int k=0; k<temp_tar_mp->track.size(); k++){
                            if(temp_tar_mp->track[k].frame->id!=matchid_2_frame[i]->id){
                                if(frame_list.count(temp_tar_mp->track[k].frame)==0){
                                    frame_list[temp_tar_mp->track[k].frame]=1;
                                }else{
                                    frame_list[temp_tar_mp->track[k].frame]=frame_list[temp_tar_mp->track[k].frame]+1;
                                }
                            }
                            
                        }
                    }
                    
                    std::vector<std::shared_ptr<gm::Frame> > connected_frames;
                    std::vector<int> connected_weights;
                    std::map<std::shared_ptr<gm::Frame>, int>::iterator it;
                    int max_count=0;
                    std::shared_ptr<gm::Frame> max_frame=nullptr;
                    for ( it = frame_list.begin(); it != frame_list.end(); it++ ){
                        if(it->second>max_count){
                            max_count=it->second;
                            max_frame=it->first;
                        }
                    }
                    if(max_count>20){
                        T_tar_sour=max_frame->getPose().inverse()*posess[i][n];
                        Eigen::Matrix3d rot=T_tar_sour.block(0,0,3,3)/scale_tar_sour;
                        Eigen::Vector3d posi=T_tar_sour.block(0,3,3,1);
                        if(matchid_2_frame[i]->id!=max_frame->id){
                            map.AddConnection(matchid_2_frame[i], max_frame, posi, rot, scale_tar_sour, max_count);
                        }
                        
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
            if(frame_inliers_kps[i][j].size()>40){
                for(int k=0; k<frame_inliers_kps[i][j].size(); k++){
                    CHECK_GT(matchid_2_frame[i]->obss.size(), frame_inliers_kps[i][j][k]);
                    CHECK_GT(map.mappoints.size(), frame_inliers_mps[i][j][k]);
                    std::shared_ptr<gm::MapPoint> temp_tar_mp = map.mappoints[frame_inliers_mps[i][j][k]];
                    bool find_same_frame=false;
                    for(int n=0; n<temp_tar_mp->track.size(); n++){
                        if(matchid_2_frame[i]->id==temp_tar_mp->track[n].frame->id){
                            find_same_frame=true;
                            break;
                        }
                    }
                    if(!find_same_frame){
                        if(matchid_2_frame[i]->obss[frame_inliers_kps[i][j][k]]==nullptr){
                            matchid_2_frame[i]->obss[frame_inliers_kps[i][j][k]]=temp_tar_mp;
                            gm::TrackItem item_t;
                            item_t.kp_ind=frame_inliers_kps[i][j][k];
                            item_t.frame=matchid_2_frame[i];
                            temp_tar_mp->track.push_back(item_t);
                            //find new pair.
                            new_match_count++;
                        }else{
                            if(matchid_2_frame[i]->obss[frame_inliers_kps[i][j][k]]->id==temp_tar_mp->id){
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
    map.FilterTrack();
    map.CalConnections();
}
