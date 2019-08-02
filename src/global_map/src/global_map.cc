#include "global_map/global_map.h"
#include <glog/logging.h>

namespace gm{
    
    std::shared_ptr<MapPoint> GlobalMap::getMPById(long unsigned int id){
        for(size_t i=0; i<mappoints.size(); i++){
            if(id==mappoints[i]->id){
                return mappoints[i];
            }
        }
        return nullptr;
    }
    
    std::shared_ptr<Frame> GlobalMap::getFrameById(long unsigned int id){
        for(size_t i=0; i<frames.size(); i++){
            if(id==frames[i]->id){
                return frames[i];
            }
        }
        return nullptr;
    }
    
    void gm::GlobalMap::CalConnections(){
        
        for(int i=0; i<frames.size(); i++){
            std::map<std::shared_ptr<Frame>, int> frame_list;
            for(int j=0; j<frames[i]->obss.size(); j++){
                if(frames[i]->obss[j]!= nullptr){
                    std::shared_ptr<MapPoint> temp_tar_mp = frames[i]->obss[j];
                    for(int k=0; k<temp_tar_mp->track.size(); k++){
                        if(frame_list.count(temp_tar_mp->track[k].frame)==0){
                            frame_list[temp_tar_mp->track[k].frame]=1;
                        }else{
                            frame_list[temp_tar_mp->track[k].frame]=frame_list[temp_tar_mp->track[k].frame]+1;
                        }
                    }
                }
                
            }
            for(std::map<std::shared_ptr<Frame>, int>::iterator it=frame_list.begin(); it!=frame_list.end(); it++){
                if(it->second>20){
                    Eigen::Matrix4d T_2_1=frames[i]->getPose().inverse()*it->first->getPose();
                    Eigen::Matrix3d rot=T_2_1.block(0,0,3,3);
                    Eigen::Vector3d posi=T_2_1.block(0,3,3,1);
                    AddConnection(it->first, frames[i] , posi, rot, 1, it->second*0.5);
                }
            }
        }
    }

    
    void GlobalMap::AddConnection(std::shared_ptr<Frame> v1, std::shared_ptr<Frame> v2, Eigen::Vector3d& posi, Eigen::Matrix3d& rot,
        double scale, double weight){
        bool dup=false;
        for(int i=0; i<pose_graph_v1.size(); i++){
            if(pose_graph_v1[i]->id== v1->id && pose_graph_v2[i]->id== v2->id){
                dup=true;
                break;
            }
        }
        if(dup){
            return;
        }
        pose_graph_v1.push_back(v1);
        pose_graph_v2.push_back(v2);
        pose_graph_weight.push_back(weight);
        pose_graph_e_scale.push_back(scale);
        pose_graph_e_posi.push_back(posi);
        pose_graph_e_rot.push_back(rot);
    }
    
    void GlobalMap::UpdatePoseEdge(){
        for(size_t i=0; i<pose_graph_v1.size(); i++){
            Eigen::Matrix4d r_pose = pose_graph_v2[i]->getPose().inverse()*pose_graph_v1[i]->getPose();
            pose_graph_e_posi[i]=r_pose.block(0,3,3,1);
            pose_graph_e_rot[i]=r_pose.block(0,0,3,3);
        }
    }
    
    void GlobalMap::DelMappoint(long unsigned int id){
        for(size_t i=0; i<mappoints.size(); i++){
            if(mappoints[i]->id==id){
                std::shared_ptr<MapPoint> mp_p = mappoints[i];
                for(size_t j=0; j<mp_p->track.size(); j++){
                    std::shared_ptr<Frame> frame_p = mp_p->track[j].frame;
                    CHECK_GT(frame_p->obss.size(), mp_p->track[j].kp_ind);
                    frame_p->obss[mp_p->track[j].kp_ind]=nullptr; 
                }
                mappoints.erase(mappoints.begin()+i);
                break;
            }
        }
    }
    
    void GlobalMap::DelFrame(long unsigned int id){
        for(size_t i=0; i<frames.size(); i++){
            if(frames[i]->id==id){
                for(int j=(int)pose_graph_v1.size()-1; j>=0; j--){
                    if(pose_graph_v1[j]->id==id || pose_graph_v2[j]->id==id){
                        pose_graph_v1.erase(pose_graph_v1.begin()+j);
                        pose_graph_v2.erase(pose_graph_v2.begin()+j);
                        pose_graph_e_posi.erase(pose_graph_e_posi.begin()+j);
                        pose_graph_e_rot.erase(pose_graph_e_rot.begin()+j);
                        pose_graph_e_scale.erase(pose_graph_e_scale.begin()+j);
                        pose_graph_weight.erase(pose_graph_weight.begin()+j);
                    }
                }
                for(size_t j=0; j<frames.size(); j++){
                    if(frames[j]->imu_next_frame!=nullptr){
                        if( frames[j]->imu_next_frame->id==id ){
                            if(frames[i]->imu_next_frame==nullptr){
                                frames[j]->imu_next_frame=nullptr;
                            }else{
                                frames[j]->imu_next_frame=frames[i]->imu_next_frame;
                                frames[j]->acces.insert(frames[j]->acces.end(), frames[i]->acces.begin(), frames[i]->acces.end());
                                frames[j]->gyros.insert(frames[j]->gyros.end(), frames[i]->gyros.begin(), frames[i]->gyros.end());
                                frames[j]->imu_times.insert(frames[j]->imu_times.end(), frames[i]->imu_times.begin(), frames[i]->imu_times.end());
                            }
                        }
                    }
                    
                }
                frames[i]->id=-1;
                frames.erase(frames.begin()+i);
            }
            
        }
        
    }
    
    void GlobalMap::GetMPPosiList(std::vector<Eigen::Vector3d>& mp_posis){
        mp_posis.resize(mappoints.size());
        for(size_t i=0; i<mappoints.size(); i++){
            mp_posis[i]=mappoints[i]->position;
        }
    }
    
    void GlobalMap::AssignKpToMp(){
        for(size_t i=0; i<mappoints.size(); i++){
            mappoints[i]->track.clear();
        }
        for(size_t i=0; i<frames.size(); i++){
            for(size_t k=0; k<frames[i]->obss.size(); k++){
                if(frames[i]->obss[k]!=nullptr){
                    TrackItem temp_track;
                    temp_track.frame=frames[i];
                    temp_track.kp_ind= k;
                    frames[i]->obss[k]->track.push_back(temp_track);
                }
            }
        }
    }
    
    void GlobalMap::CalPoseEdgeVal(){
        // for (size_t i=0; i<pose_graph_v1.size(); i++){
        //     Eigen::Matrix4d rel_pose= pose_graph_v2[i]->getPose().inverse() *pose_graph_v1[i]->getPose();
        // }
    }
    
    void GlobalMap::CheckConsistence(){
        for(size_t i=0; i<mappoints.size(); i++){
            for(size_t j=0; j<mappoints[i]->track.size(); j++){
                std::shared_ptr< Frame> frame_p = mappoints[i]->track[j].frame;
                if(frame_p==nullptr){
                    LOG(INFO)<<"frame_p==nullptr";
                }
                std::shared_ptr< MapPoint> mp = frame_p->obss[mappoints[i]->track[j].kp_ind];
                if(mp==nullptr){
                    LOG(INFO)<<"mp==nullptr";
                }else{
                    if(mp->id!=mappoints[i]->id){
                        LOG(INFO)<<"mp->id!=mappoints[i]->id: "<<mp->id<<" : "<<mappoints[i]->id;
                    }
                }
            }       
        }
        
        for(size_t i=0; i<frames.size(); i++){
            for(size_t j=0; j<frames[i]->obss.size(); j++){
                if(frames[i]->obss[j]!=nullptr){
                    std::shared_ptr< MapPoint> mp=frames[i]->obss[j];
                    bool find_one=false;
                    for(size_t k=0; k<mp->track.size(); k++){
                        if(mp->track[k].frame->id==frames[i]->id && (size_t)mp->track[k].kp_ind==j){
                            find_one=true;
                        }
                    }
                    if(find_one==false){
                        LOG(INFO)<<"find_one==false";
                        frames[i]->obss[j]==nullptr;
                    }
                }
            }
        }
    }
    
    void GlobalMap::GetCovisi(std::shared_ptr< Frame> frame_p, std::map<std::shared_ptr< Frame>, int>& connections){
        for(int j=0 ; j<frame_p->obss.size(); j++){
            if(frame_p->obss[j]!=nullptr){
                for(int k=0; k<frame_p->obss[j]->track.size(); k++){
                    std::shared_ptr< Frame> temp_frame_p = frame_p->obss[j]->track[k].frame;
                    CHECK_NOTNULL(temp_frame_p);
                    if(temp_frame_p->id==-1){
                        continue;
                    }
                    if(temp_frame_p->id==frame_p->id){
                        continue;
                    }
                    if(connections.count(temp_frame_p)==0){
                        connections[temp_frame_p]=1;
                    }else{
                        connections[temp_frame_p]=connections[temp_frame_p]+1;
                    }
                }
            }
        }
    }
    
    void GlobalMap::CreateSubMap(int startframe_id, int endframe_id, GlobalMap& submap){
        submap=*(this);
        submap.frames.clear();
        submap.mappoints.clear();
        std::map<std::shared_ptr<gm::Frame>, std::shared_ptr<gm::Frame>> old_to_new_frame_map;
        for(int i=startframe_id; i<endframe_id; i++){
            std::shared_ptr<gm::Frame> frame_p;
            frame_p.reset(new gm::Frame);
            *(frame_p)=*(frames[i]);
            for(size_t j=0; j<frame_p->obss.size(); j++){
                frame_p->obss[j]=nullptr;
            }
            submap.frames.push_back(frame_p);
            old_to_new_frame_map[frames[i]]=frame_p;
        }
        for(size_t i=0; i<submap.frames.size(); i++){
            if(submap.frames[i]->imu_next_frame!=nullptr){
                int old_frame_id=submap.frames[i]->imu_next_frame->id;
                if(old_to_new_frame_map.count(submap.frames[i]->imu_next_frame)!=0){
                    submap.frames[i]->imu_next_frame=old_to_new_frame_map[submap.frames[i]->imu_next_frame];
                }else{
                    submap.frames[i]->imu_next_frame=nullptr;
                    submap.frames[i]->acces.clear();
                    submap.frames[i]->gyros.clear();
                    submap.frames[i]->imu_times.clear();
                }
            }
        }
        
        for(size_t i=0; i<mappoints.size(); i++){
            for(size_t j=0; j<mappoints[i]->track.size(); j++){
                if(old_to_new_frame_map.count(mappoints[i]->track[j].frame)!=0){
                    std::shared_ptr<gm::MapPoint> mappoint_p= submap.getMPById(mappoints[i]->id);
                    if(mappoint_p==nullptr){
                        mappoint_p.reset(new gm::MapPoint);
                        *(mappoint_p)=*(mappoints[i]);
                        mappoint_p->track.clear();
                        submap.mappoints.push_back(mappoint_p);
                    }
                    std::shared_ptr<gm::Frame> new_frame_temp=old_to_new_frame_map[mappoints[i]->track[j].frame];
                    new_frame_temp->obss[mappoints[i]->track[j].kp_ind]=mappoint_p;                
                }
            }
        }
        submap.AssignKpToMp();
    }
    
    void GlobalMap::FilterTrack(){
        int dup_count=0;
        for(int i=0; i<mappoints.size(); i++){
            std::map<std::shared_ptr< Frame>, std::vector<int>> frame_mask;
            for(int j=0; j<mappoints[i]->track.size(); j++){
                if(frame_mask.count(mappoints[i]->track[j].frame)!=0){
                    frame_mask[mappoints[i]->track[j].frame].push_back(j);
                    dup_count++;
                }else{
                    std::vector<int> temp;
                    temp.push_back(j);
                    frame_mask[mappoints[i]->track[j].frame]=temp;
                }
            }
            std::map<std::shared_ptr< Frame>, std::vector<int>>::iterator it;
            for ( it = frame_mask.begin(); it != frame_mask.end(); it++ ){
                if(it->second.size()>1){
                    float min_err=11111;
                    int min_trackid;
                    //std::cout<<"++++++++++++++++++++++"<<std::endl;
                    for(int j=0; j<it->second.size(); j++){
                        Eigen::Matrix<double, 3,4> proj_mat = it->first->getProjMat();
                        Eigen::Vector4d posi_homo;
                        posi_homo.block(0,0,3,1)=mappoints[i]->position;
                        posi_homo(3)=1;
                        Eigen::Vector3d proj_homo = proj_mat*posi_homo;
                        double u=proj_homo(0)/proj_homo(2);
                        double v=proj_homo(1)/proj_homo(2);
                        int kp_ind = mappoints[i]->track[it->second[j]].kp_ind;
                        cv::Point2f uv= it->first->kps[kp_ind].pt;
                        float proj_err=sqrt((uv.x-u)*(uv.x-u)+(uv.y-v)*(uv.y-v));
                        //std::cout<<kp_ind<<":"<<proj_err<<std::endl;
                        if(proj_err<min_err){
                            min_err=proj_err;
                            min_trackid=it->second[j];
                        }
                    }
                    for(int j=0; j<it->second.size(); j++){
                        if(it->second[j]!=min_trackid){
                            int kp_ind = mappoints[i]->track[it->second[j]].kp_ind;
                            //std::cout<<"del: "<<kp_ind<<std::endl;
                            it->first->obss[kp_ind]=nullptr;
                        }
                    }
                }
            }
        }
        std::cout<<"dup_count: "<<dup_count<<std::endl;
    }
}