#include <backend/header.h>

DEFINE_string(project_mat_file, "", "");

void MergeMP(std::shared_ptr<gm::MapPoint> base_mp, std::shared_ptr<gm::MapPoint> to_merge_mp){
    for(int k=0; k<to_merge_mp->track.size(); k++){
        CHECK_GT(to_merge_mp->track[k].frame->obss.size(), to_merge_mp->track[k].kp_ind);
        to_merge_mp->track[k].frame->obss[to_merge_mp->track[k].kp_ind]=base_mp;
    }
}

void update_corresponds(std::shared_ptr<gm::GlobalMap> map_p){
    chamo::GlobalMatch global_matcher;
    global_matcher.LoadMap(FLAGS_project_mat_file, map_p);
    std::vector<std::vector<std::vector<int>>> frame_inliers_mps;
    std::vector<std::vector<std::vector<int>>> frame_inliers_kps;
    for(int i=0; i<map_p->frames.size(); i++){
        std::vector<std::vector<int>> inliers_mps;
        std::vector<std::vector<int>> inliers_kps;
        std::vector<Eigen::Matrix4d> poses;
        global_matcher.MatchImg(map_p->frames[i], inliers_mps, inliers_kps, poses);
        frame_inliers_mps.push_back(inliers_mps);
        frame_inliers_kps.push_back(inliers_kps);
        for(int n=0; n<poses.size(); n++){
            if(inliers_kps[n].size()>30){
                std::vector<Eigen::Vector3d> local_pc;
                std::vector<Eigen::Vector3d> global_pc;
                for(int j=0; j<inliers_kps[n].size(); j++){
                    if(map_p->frames[i]->obss[inliers_kps[n][j]]!=nullptr){
                        local_pc.push_back(map_p->frames[i]->obss[inliers_kps[n][j]]->position);
                        global_pc.push_back(map_p->mappoints[inliers_mps[n][j]]->position);
                    }
                }
                Eigen::Matrix4d T_tar_sour;
                double scale_tar_sour; 
                
                bool succ= chamo::ComputeSim3Ransac(global_pc, local_pc, T_tar_sour, scale_tar_sour);
                if(succ){
                    //LOG(INFO)<<"scale_tar_sour: "<<scale_tar_sour;
                    std::map<std::shared_ptr<gm::Frame>, int> frame_list;
                    for(int j=0; j<inliers_mps[n].size(); j++){
                        std::shared_ptr<gm::MapPoint> temp_tar_mp = map_p->mappoints[inliers_mps[n][j]];
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
                        Eigen::Matrix4d T_sourworld_sour = map_p->frames[i]->getPose();
                        Eigen::Matrix4d T_tar_tarworld = T_tarworld_tar.inverse();
                        Eigen::Matrix4d T_tar_sour = T_tar_tarworld*T_tarworld_sourworld*T_sourworld_sour;
                        Eigen::Matrix3d rot=T_tar_sour.block(0,0,3,3)/scale_tar_sour;
                        Eigen::Vector3d posi=T_tar_sour.block(0,3,3,1);
                        map_p->AddConnection(map_p->frames[i], connected_frames[j], posi, rot, scale_tar_sour, max_count);

                    }
    //                 Eigen::Matrix4d T_sourworld_sour = source_map_p->frames[i]->getPose();
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
                //std::cout<<"not enouph local_pc!!"<<std::endl;
                //return false;
            }
        }
    }
    std::vector<std::set<long unsigned int >> to_merge_mps;
    for(int i=0; i<frame_inliers_kps.size(); i++){
        for(int j=0; j<frame_inliers_kps[i].size(); j++){
            if(frame_inliers_kps[i][j].size()>10){
                for(int k=0; k<frame_inliers_kps[i][j].size(); k++){
                    if(map_p->frames[i]->obss[frame_inliers_kps[i][j][k]]==nullptr){
                        map_p->frames[i]->obss[frame_inliers_kps[i][j][k]]=map_p->mappoints[frame_inliers_mps[i][j][k]];
                        //find new pair.
                    }else{
                        if(map_p->frames[i]->obss[frame_inliers_kps[i][j][k]]->id==map_p->mappoints[frame_inliers_mps[i][j][k]]->id){
                            //find the same pair, do nothing.
                        }else{
                            bool find_one=false;
                            for(int n=0; n<to_merge_mps.size(); n++){
                                auto search = to_merge_mps[n].find(map_p->frames[i]->obss[frame_inliers_kps[i][j][k]]->id);
                                if (search != to_merge_mps[n].end()) {
                                    to_merge_mps[n].insert(map_p->frames[i]->obss[frame_inliers_kps[i][j][k]]->id);
                                    to_merge_mps[n].insert(map_p->mappoints[frame_inliers_mps[i][j][k]]->id);
                                    find_one=true;
                                }
                                search = to_merge_mps[n].find(map_p->mappoints[frame_inliers_mps[i][j][k]]->id);
                                if (search != to_merge_mps[n].end()) {
                                    to_merge_mps[n].insert(map_p->frames[i]->obss[frame_inliers_kps[i][j][k]]->id);
                                    to_merge_mps[n].insert(map_p->mappoints[frame_inliers_mps[i][j][k]]->id);
                                    find_one=true;
                                }
                            }
                            if(find_one==false){
                                std::set<long unsigned int> temp_set;
                                temp_set.insert(map_p->frames[i]->obss[frame_inliers_kps[i][j][k]]->id);
                                temp_set.insert(map_p->mappoints[frame_inliers_mps[i][j][k]]->id);
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
    for(int i=0; i<map_p->mappoints.size(); i++){
        mpid_to_vecter_id[map_p->mappoints[i]->id]=i;
        to_dels.push_back(false);
    }
    for(int i=0; i<to_merge_mps.size(); i++){
        //std::cout<<"merged count: "<<to_merge_mps[i].size()<<std::endl;
        std::set<long unsigned int>::iterator it;
        std::shared_ptr<gm::MapPoint> first_mp=nullptr;
        for (it = to_merge_mps[i].begin(); it != to_merge_mps[i].end(); ++it){
            if(first_mp!=nullptr){
                MergeMP(first_mp, map_p->mappoints[mpid_to_vecter_id[*it]]);
                to_dels[mpid_to_vecter_id[*it]]=true;
            }else{
                first_mp=map_p->mappoints[mpid_to_vecter_id[*it]];
            }
        }
    }
    for(int i=to_dels.size()-1; i>=0; i--){
        map_p->mappoints.erase(map_p->mappoints.begin()+i);
    }
    map_p->AssignKpToMp();
}