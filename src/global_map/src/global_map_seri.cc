#include "global_map/global_map_seri.h"
#include "separate_map.h"
#include <glog/logging.h>
#include "CoorConv.h"
#include <unordered_map>

namespace gm{
    void putToFile(float val, std::ofstream& f){
        f.write((char*)&val, 4);
    }
    float getFromFileF(std::ifstream& f){
        float val;
        f.read((char*)&val, 4);
        return val;
    }
    void putToFile(double val, std::ofstream& f){
        f.write((char*)&val, 8);
    }
    double getFromFileD(std::ifstream& f){
        double val;
        f.read((char*)&val, 8);
        return val;
    }
    void putToFileD2F(float val, std::ofstream& f){
        float val_f=(float)val;
        f.write((char*)&val_f, 4);
    }
    double getFromFileF2D(std::ifstream& f){
        float val_f;
        f.read((char*)&val_f, 4);
        return val_f;
    }
    void putToFile(int val, std::ofstream& f){
        f.write((char*)&val, 4);
    }
    int getFromFileI(std::ifstream& f){
        int val;
        f.read((char*)&val, 4);
        return val;
    }
    void putToFile(long unsigned int val, std::ofstream& f){
        f.write((char*)&val, 8);
    }
    long unsigned int getFromFileLI(std::ifstream& f){
        long unsigned int val;
        f.read((char*)&val, 8);
        return val;
    }
    void putToFile(std::string& str, std::ofstream& f){
        int str_len=str.size();
        f.write((char*)&str_len, 4);
        f.write((const char*)str.c_str(), str.size());
    }
    std::string getFromFileS(std::ifstream& f){
        std::string str;
        int str_len;
        f.read((char*)&str_len, 4);
        CHECK_GT(100, str_len);
        for(int i=0; i<str_len; i++){
            char c;
            f.read(&c, 1);
            str.push_back(c);
        }
        return str;
    }

    void save_submap(GlobalMap& map, std::string file_addr){
        std::ofstream output(file_addr, std::ios::out | std::ios::trunc | std::ios::binary);
        putToFile((int)map.mappoints.size(), output);
        std::cout<<"mp count: "<<map.mappoints.size()<<std::endl;
        for(size_t i=0; i<map.mappoints.size(); i++){
            std::shared_ptr<MapPoint> mappoint_p = map.mappoints[i];
            putToFile(mappoint_p->id, output);
            putToFileD2F(mappoint_p->position.x(), output);
            putToFileD2F(mappoint_p->position.y(), output);
            putToFileD2F(mappoint_p->position.z(), output);
            //putToFile(mappoint_p->match_count, output);
            //std::cout<<mappoint_p->position.transpose()<<std::endl;
        }
        
        putToFile((int)map.frames.size(), output);
        std::cout<<"frame count: "<<map.frames.size()<<std::endl;
        int kp_count=0;
        for(size_t i=0; i<map.frames.size(); i++){
            std::shared_ptr<Frame> frame_p=map.frames[i];
            putToFile(frame_p->id, output);
            putToFile(frame_p->frame_file_name, output);
            
            putToFile(frame_p->time_stamp, output);
            
            putToFile((int)frame_p->doMatch, output);
            putToFile((int)frame_p->doGraphOpti, output);
            putToFile((int)frame_p->doBA, output);
            putToFile((int)frame_p->isfix, output);
            
            putToFileD2F(frame_p->Tbc_posi.x(), output);
            putToFileD2F(frame_p->Tbc_posi.y(), output);
            putToFileD2F(frame_p->Tbc_posi.z(), output);
            putToFileD2F(frame_p->Tbc_qua.w(), output);
            putToFileD2F(frame_p->Tbc_qua.x(), output);
            putToFileD2F(frame_p->Tbc_qua.y(), output);
            putToFileD2F(frame_p->Tbc_qua.z(), output);
            
            putToFileD2F(frame_p->position.x(), output);
            putToFileD2F(frame_p->position.y(), output);
            putToFileD2F(frame_p->position.z(), output);
            putToFileD2F(frame_p->direction.w(), output);
            putToFileD2F(frame_p->direction.x(), output);
            putToFileD2F(frame_p->direction.y(), output);
            putToFileD2F(frame_p->direction.z(), output);
            
            putToFile(frame_p->fx, output);
            putToFile(frame_p->fy, output);
            putToFile(frame_p->cx, output);
            putToFile(frame_p->cy, output);
            putToFile(frame_p->k1, output);
            putToFile(frame_p->k2, output);
            putToFile(frame_p->p1, output);
            putToFile(frame_p->p2, output);
            putToFile(frame_p->width, output);
            putToFile(frame_p->height, output);
            
            putToFileD2F(frame_p->gps_position.x(), output);
            putToFileD2F(frame_p->gps_position.y(), output);
            putToFileD2F(frame_p->gps_position.z(), output);
            putToFile(frame_p->gps_accu, output);
            putToFile(frame_p->gps_avg_count, output);

            std::vector<cv::KeyPoint>& keypoints1=frame_p->kps;
            putToFile((int)keypoints1.size(), output);
            for (size_t j=0; j<keypoints1.size(); j++){
                kp_count++;
                putToFile(keypoints1[j].pt.x, output);
                putToFile(keypoints1[j].pt.y, output);
                if(frame_p->isborder==true){
                    putToFile(frame_p->obss_ids[j], output);
                }else{
                    if(frame_p->obss[j]!=nullptr){
                        putToFile(frame_p->obss[j]->id, output);
                    }else{
                        putToFile((long unsigned int)-1, output);
                    }
                }
                
                putToFile(keypoints1[j].octave, output);
            }
            int desc_width=frame_p->descriptors.rows();
            int desc_count=frame_p->descriptors.cols();
            putToFile(desc_width, output);
            putToFile(desc_count, output);
            for(int j=0; j<desc_count; j++){
                for(int k=0; k<desc_width; k++){
                    output.write( (const char*)&frame_p->descriptors(k, j), 1 );
                }
            }
            putToFile((int)frame_p->acces.size(), output);
            for(size_t j=0; j<frame_p->acces.size(); j++){
                putToFileD2F(frame_p->acces[j](0), output);
                putToFileD2F(frame_p->acces[j](1), output);
                putToFileD2F(frame_p->acces[j](2), output);
                putToFileD2F(frame_p->gyros[j](0), output);
                putToFileD2F(frame_p->gyros[j](1), output);
                putToFileD2F(frame_p->gyros[j](2), output);
                putToFile(frame_p->imu_times[j], output);
            }
            if(frame_p->isborder==true){
                putToFile(frame_p->imu_next_frame_id, output);
            }else{
                if(frame_p->imu_next_frame==nullptr){
                    putToFile((long unsigned int)-1, output);
                }else{
                    putToFile(frame_p->imu_next_frame->id, output);
                }
            }
        }
        std::cout<<"kp count: "<<kp_count<<std::endl;
        
        putToFile((int)map.pose_graph_e_posi.size(), output);
        std::cout<<"covisi count: "<<map.pose_graph_e_posi.size()<<std::endl;
        CHECK_EQ(map.pose_graph_e_posi.size(),map.pose_graph_e_posi.size());
        CHECK_EQ(map.pose_graph_e_rot.size(),map.pose_graph_e_posi.size());
        CHECK_EQ(map.pose_graph_e_scale.size(),map.pose_graph_e_posi.size());
        CHECK_EQ(map.pose_graph_weight.size(),map.pose_graph_e_posi.size());
        CHECK_EQ(map.pose_graph_v1.size(),map.pose_graph_e_posi.size());
        CHECK_EQ(map.pose_graph_v2.size(),map.pose_graph_e_posi.size());
        for(size_t i=0; i<map.pose_graph_e_posi.size(); i++){
            putToFileD2F(map.pose_graph_e_posi[i].x(), output);
            putToFileD2F(map.pose_graph_e_posi[i].y(), output);
            putToFileD2F(map.pose_graph_e_posi[i].z(), output);
            Eigen::Quaterniond rot_qua(map.pose_graph_e_rot[i]);
            putToFileD2F(rot_qua.w(), output);
            putToFileD2F(rot_qua.x(), output);
            putToFileD2F(rot_qua.y(), output);
            putToFileD2F(rot_qua.z(), output);
            putToFileD2F(map.pose_graph_e_scale[i], output);
            putToFileD2F(map.pose_graph_weight[i], output);
            putToFile(map.pose_graph_v1[i]->id, output);
            putToFile(map.pose_graph_v2[i]->id, output);
        }
        output.close();
        map.ReleaseMap();
    }
    
    void read_mpkf_count(int& mp_count, int& kf_count, std::string file_addr, std::vector<unsigned int> ids){
        if(ids.size()==0){
            return;
        }
        mp_count=0;
        kf_count=0;
        for(int i=0; i<ids.size(); i++){
            unsigned int map_id_temp=ids[i];
            std::stringstream ss;
            ss<<map_id_temp;
            std::string full_file_name=file_addr+"/"+ss.str()+".map";
            std::ifstream input(full_file_name.c_str(), std::ios::binary);
            if(!input.is_open()){
                continue;
            }
            mp_count=mp_count+getFromFileI(input);
            input.seekg ( mp_count*20, input.cur);
            kf_count=kf_count+getFromFileI(input);
            input.close();
        }
    }
    
    void fast_load_mps(std::vector<Eigen::Vector3d>& mp_posis, std::vector<Eigen::Vector3d>& kf_posis, std::vector<Eigen::Quaterniond>& kf_rot, std::string file_addr, std::vector<unsigned int> ids){
        if(ids.size()==0){
            return;
        }
        Eigen::Vector3d gps_anchor;
        get_gps_from_block_id(gps_anchor, ids[0]);
        for(int i=0; i<ids.size(); i++){
            unsigned int map_id_temp=ids[i];
            Eigen::Vector3d cur_gps_anchor;
            get_gps_from_block_id(cur_gps_anchor, map_id_temp);
            std::stringstream ss;
            ss<<map_id_temp;
            std::string full_file_name=file_addr+"/"+ss.str()+".map";
            std::ifstream input(full_file_name.c_str(), std::ios::binary);
            if(!input.is_open()){
                continue;
            }
            int mappoints_size=getFromFileI(input);
            for(int i=0; i<mappoints_size; i++){
                getFromFileLI(input);
                Eigen::Vector3d posi;
                posi.x()=getFromFileF2D(input);
                posi.y()=getFromFileF2D(input);
                posi.z()=getFromFileF2D(input);
                Eigen::Vector3d out_tar_xyz;
                convert_to_another_anchor(cur_gps_anchor, gps_anchor, posi, out_tar_xyz);
                mp_posis.push_back(out_tar_xyz);
            }
            int frames_size;
            frames_size = getFromFileI(input);
            for(int i=0; i<frames_size; i++){
                input.seekg ( 8, input.cur);
                getFromFileS(input);
                input.seekg ( 13*4, input.cur);
                Eigen::Vector3d temp_posi;
                Eigen::Quaterniond temp_qua;
                temp_posi.x()=getFromFileF2D(input);
                temp_posi.y()=getFromFileF2D(input);
                temp_posi.z()=getFromFileF2D(input);
                temp_qua.w()=getFromFileF2D(input);
                temp_qua.x()=getFromFileF2D(input);
                temp_qua.y()=getFromFileF2D(input);
                temp_qua.z()=getFromFileF2D(input);
                Eigen::Vector3d out_tar_xyz;
                convert_to_another_anchor(cur_gps_anchor, gps_anchor, temp_posi, out_tar_xyz);
                kf_posis.push_back(out_tar_xyz);
                kf_rot.push_back(temp_qua);
                input.seekg ( 15*4, input.cur);
                int kps_size;
                kps_size=getFromFileI(input);
                input.seekg ( kps_size*4*5, input.cur);
                int desc_width=getFromFileI(input);
                int desc_count=getFromFileI(input);
                input.seekg ( desc_width*desc_count, input.cur);
                int imu_count=getFromFileI(input);
                input.seekg ( imu_count*8*4+8, input.cur);
            }
            input.close();
        }
    }


    bool load_submap(GlobalMap& map, std::string file_addr, bool do_recover_obs){
        std::ifstream input(file_addr.c_str(), std::ios::in | std::ios::binary);
        if(!input.is_open()){
            return false;
        }
        int mappoints_size=getFromFileI(input);
        std::cout<<"mp count: "<<mappoints_size<<std::endl;
        for(int i=0; i<mappoints_size; i++){
            std::shared_ptr< MapPoint> mappoint_p;
            mappoint_p.reset(new  MapPoint);
            mappoint_p->id = getFromFileLI(input);
            mappoint_p->position.x()=getFromFileF2D(input);
            mappoint_p->position.y()=getFromFileF2D(input);
            mappoint_p->position.z()=getFromFileF2D(input);
            //mappoint_p->match_count=getFromFileI(input);
            //std::cout<<mappoint_p->id<<std::endl;
            map.mappoints.push_back(mappoint_p);
        }
        
        int frames_size;
        frames_size = getFromFileI(input);
        std::cout<<"frame count: "<<frames_size<<std::endl;
        int kp_count=0;
        std::vector<int> nextimu_frameids;
        for(int i=0; i<frames_size; i++){
            std::shared_ptr< Frame> frame_p;
            frame_p.reset(new  Frame);
            frame_p->id = getFromFileLI(input);
            
            frame_p->frame_file_name = getFromFileS(input);
            frame_p->time_stamp=getFromFileD(input);
            
            frame_p->doMatch=getFromFileI(input);
            frame_p->doGraphOpti=getFromFileI(input);
            frame_p->doBA=getFromFileI(input);
            frame_p->isfix=getFromFileI(input);
            
            frame_p->Tbc_posi.x()=getFromFileF2D(input);
            frame_p->Tbc_posi.y()=getFromFileF2D(input);
            frame_p->Tbc_posi.z()=getFromFileF2D(input);
            frame_p->Tbc_qua.w()=getFromFileF2D(input);
            frame_p->Tbc_qua.x()=getFromFileF2D(input);
            frame_p->Tbc_qua.y()=getFromFileF2D(input);
            frame_p->Tbc_qua.z()=getFromFileF2D(input);
            
            frame_p->position.x()=getFromFileF2D(input);
            frame_p->position.y()=getFromFileF2D(input);
            frame_p->position.z()=getFromFileF2D(input);
            frame_p->direction.w()=getFromFileF2D(input);
            frame_p->direction.x()=getFromFileF2D(input);
            frame_p->direction.y()=getFromFileF2D(input);
            frame_p->direction.z()=getFromFileF2D(input);
            
            frame_p->fx=getFromFileF(input);
            frame_p->fy=getFromFileF(input);
            frame_p->cx=getFromFileF(input);
            frame_p->cy=getFromFileF(input);
            frame_p->k1=getFromFileF(input);
            frame_p->k2=getFromFileF(input);
            frame_p->p1=getFromFileF(input);
            frame_p->p2=getFromFileF(input);
            frame_p->width=getFromFileI(input);
            frame_p->height=getFromFileI(input);
            frame_p->gps_position.x()=getFromFileF2D(input);
            frame_p->gps_position.y()=getFromFileF2D(input);
            frame_p->gps_position.z()=getFromFileF2D(input);
            frame_p->gps_accu=getFromFileF(input);
            frame_p->gps_avg_count=getFromFileI(input);
            int kps_size;
            kps_size=getFromFileI(input);
            
            frame_p->obss.resize(kps_size);
            frame_p->obss_ids.resize(kps_size);
            for(int j=0; j<kps_size; j++){
                kp_count++;
                cv::KeyPoint kp;
                kp.pt.x=getFromFileF(input);
                kp.pt.y=getFromFileF(input);
                frame_p->obss_ids[j]=getFromFileLI(input);
                kp.octave = getFromFileI(input);
                frame_p->kps.push_back(kp);
            }
            int desc_width=getFromFileI(input);
            int desc_count=getFromFileI(input);
            frame_p->descriptors.resize(desc_width, desc_count);
            for(int j=0; j<desc_count; j++){
                for(int k=0; k<desc_width; k++){
                    char temp_c;
                    input.read(&temp_c, 1 );
                    frame_p->descriptors(k, j)=temp_c;
                }
            }
            
            int imu_count=getFromFileI(input);
            for(int j=0; j<imu_count; j++){
                Eigen::Vector3d acce;
                acce(0)=getFromFileF2D(input);
                acce(1)=getFromFileF2D(input);
                acce(2)=getFromFileF2D(input);
                Eigen::Vector3d gyro;
                gyro(0)=getFromFileF2D(input);
                gyro(1)=getFromFileF2D(input);
                gyro(2)=getFromFileF2D(input);
                frame_p->acces.push_back(acce);
                frame_p->gyros.push_back(gyro);
                frame_p->imu_times.push_back(getFromFileD(input));
            }
            frame_p->imu_next_frame_id=getFromFileLI(input);
            map.frames.push_back(frame_p);
        }
        std::cout<<"kp count: "<<kp_count<<std::endl;
        
        int pose_graph_e_size;
        pose_graph_e_size=getFromFileI(input);
        for(int i=0; i<pose_graph_e_size; i++){
            Eigen::Vector3d posi;
            posi.x()=getFromFileF2D(input);
            posi.y()=getFromFileF2D(input);
            posi.z()=getFromFileF2D(input);
            Eigen::Quaterniond rot_qua;
            rot_qua.w()=getFromFileF2D(input);
            rot_qua.x()=getFromFileF2D(input);
            rot_qua.y()=getFromFileF2D(input);
            rot_qua.z()=getFromFileF2D(input);
            
            double temp_scale=getFromFileF2D(input);
            double temp_weight=getFromFileF2D(input);
            
            long unsigned int v1_id =getFromFileLI(input);
            long unsigned int v2_id =getFromFileLI(input);
            std::shared_ptr<Frame> frame1_p = map.getFrameById(v1_id);
            std::shared_ptr<Frame> frame2_p = map.getFrameById(v2_id);
            if(frame1_p!=nullptr && frame2_p!=nullptr){
                map.pose_graph_e_posi.push_back(posi);
                Eigen::Matrix3d rot(rot_qua);
                map.pose_graph_e_rot.push_back(rot);
                map.pose_graph_e_scale.push_back(temp_scale);
                map.pose_graph_weight.push_back(temp_weight);
                map.pose_graph_v1.push_back(frame1_p);
                map.pose_graph_v2.push_back(frame2_p);
            }
            
        }
        std::cout<<"covisi count: "<<pose_graph_e_size<<std::endl;
        if(do_recover_obs){
            std::unordered_map<long unsigned int, std::shared_ptr<MapPoint>> temp_mp_map;
            for(int i=0; i<map.mappoints.size(); i++){
                temp_mp_map[map.mappoints[i]->id]=map.mappoints[i];
            }
            for(int i=0; i<map.frames.size(); i++){
                if(map.frames[i]->imu_next_frame_id!=-1){
                    map.frames[i]->imu_next_frame=map.getFrameById(map.frames[i]->imu_next_frame_id);
                }
                for(int j=0; j<map.frames[i]->obss_ids.size(); j++){
                    if(map.frames[i]->obss_ids[j]!= (long unsigned int)-1){
                        std::unordered_map<long unsigned int, std::shared_ptr<MapPoint>>::const_iterator it = temp_mp_map.find(map.frames[i]->obss_ids[j]);
                        if(it!=temp_mp_map.end()){
                            map.frames[i]->obss[j]=it->second;
                        }
                        
                    }
                }
            }
        }
        input.close();
        return true;
    }
    
    void get_blockid_list(GlobalMap& map, std::vector<unsigned int>& out_block_ids){
        std::map<unsigned int, std::shared_ptr<GlobalMap>> submaps;
        for(int i=0; i<map.frames.size(); i++){
            unsigned int block_id;
            get_map_block_id_from_id(block_id, map.frames[i]->id);
            if(submaps.count(block_id)==0){
                submaps[block_id].reset(new GlobalMap);
            }
            submaps[block_id]->frames.push_back(map.frames[i]);
        }
        for(int i=0; i<map.mappoints.size(); i++){
            unsigned int block_id;
            get_map_block_id_from_id(block_id, map.mappoints[i]->id);
            if(submaps.count(block_id)==0){
                submaps[block_id].reset(new GlobalMap);
            }
            submaps[block_id]->mappoints.push_back(map.mappoints[i]);
        }
        for(std::map<unsigned int, std::shared_ptr<GlobalMap>>::iterator it=submaps.begin(); it!=submaps.end(); it++){
            out_block_ids.push_back(it->first);
        }
    }
    
    void save_global_map(GlobalMap& map, std::string file_addr){
        std::map<unsigned int, std::shared_ptr<GlobalMap>> submaps;
        
        std::cout<<"global map: "<<map.frames.size()<<std::endl;
        for(int i=0; i<map.frames.size(); i++){
            unsigned int block_id;
            get_map_block_id_from_id(block_id, map.frames[i]->id);
            if(submaps.count(block_id)==0){
                submaps[block_id].reset(new GlobalMap);
            }
            Eigen::Vector3d temp_anchor=map.gps_anchor;
            get_gps_from_block_id(temp_anchor, block_id);
            Eigen::Vector3d out_tar_xyz;
            convert_to_another_anchor(map.gps_anchor, temp_anchor, map.frames[i]->position, out_tar_xyz);
            map.frames[i]->position=out_tar_xyz;
            convert_to_another_anchor(map.gps_anchor, temp_anchor, map.frames[i]->gps_position, out_tar_xyz);
            map.frames[i]->gps_position=out_tar_xyz;
            submaps[block_id]->frames.push_back(map.frames[i]);
        }
        for(int i=0; i<map.mappoints.size(); i++){
            unsigned int block_id;
            get_map_block_id_from_id(block_id, map.mappoints[i]->id);
            if(submaps.count(block_id)==0){
                submaps[block_id].reset(new GlobalMap);
            }
            Eigen::Vector3d temp_anchor=map.gps_anchor;
            get_gps_from_block_id(temp_anchor, block_id);
            Eigen::Vector3d out_tar_xyz;
            convert_to_another_anchor(map.gps_anchor, temp_anchor, map.mappoints[i]->position, out_tar_xyz);
            map.mappoints[i]->position=out_tar_xyz;
            submaps[block_id]->mappoints.push_back(map.mappoints[i]);
        }
        for(int i=0; i<map.pose_graph_v1.size(); i++){
            //std::cout<<i<<":"<<map.pose_graph_v1.size()<<std::endl;
            unsigned int block_id;
            get_map_block_id_from_id(block_id, map.pose_graph_v1[i]->id);
            std::map<unsigned int, std::shared_ptr<GlobalMap>>::iterator find_it = submaps.find(block_id);
            if(find_it==submaps.end()){
                continue;
            }
            CHECK_GT(map.pose_graph_v1.size(),i);
            CHECK_GT(map.pose_graph_v2.size(),i);
            CHECK_GT(map.pose_graph_e_posi.size(),i);
            CHECK_GT(map.pose_graph_e_rot.size(),i);
            CHECK_GT(map.pose_graph_e_scale.size(),i);
            CHECK_GT(map.pose_graph_weight.size(),i);
            submaps[block_id]->pose_graph_v1.push_back(map.pose_graph_v1[i]);
            submaps[block_id]->pose_graph_v2.push_back(map.pose_graph_v2[i]);
            submaps[block_id]->pose_graph_e_posi.push_back(map.pose_graph_e_posi[i]);
            submaps[block_id]->pose_graph_e_rot.push_back(map.pose_graph_e_rot[i]);
            submaps[block_id]->pose_graph_e_scale.push_back(map.pose_graph_e_scale[i]);
            submaps[block_id]->pose_graph_weight.push_back(map.pose_graph_weight[i]);
        }
        for(std::map<unsigned int, std::shared_ptr<GlobalMap>>::iterator it=submaps.begin(); it!=submaps.end(); it++){
            std::stringstream ss;
            ss<<it->first;
            std::cout<<"save submap: "<<file_addr+"/"+ss.str()+".map"<<std::endl;
            save_submap(*(it->second), file_addr+"/"+ss.str()+".map");
        }
    }
    
    //std::vector<unsigned int> getNearBlock(std::vector<unsigned int>& input_ids){
    //    double min_lon=9999;
    //    double max_lon=-9999;
    //    double min_lat=9999;
    //    double max_lat=-9999;
    //    for(int i=0; i<input_ids.size(); i++){
    //        Eigen::Vector3d gps_latlon;
    //        get_gps_from_block_id(gps_latlon, input_ids[i]);
    //        //std::cout<<std::setprecision(15)<<gps_latlon(0)<<" : "<<gps_latlon(1)<<std::endl;
    //        if(gps_latlon(0)>max_lon){
    //            max_lon=gps_latlon(0);
    //        }
    //        if(gps_latlon(0)<min_lon){
    //            min_lon=gps_latlon(0);
    //        }
    //        if(gps_latlon(1)>max_lat){
    //            max_lat=gps_latlon(1);
    //        }
    //        if(gps_latlon(1)<min_lat){
    //            min_lat=gps_latlon(1);
    //        }
    //    }
    //    //std::cout<<std::setprecision(15)<<min_lon<<" : "<<max_lon<<std::endl;
    //    //std::cout<<std::setprecision(15)<<min_lat<<" : "<<max_lat<<std::endl;

    //    min_lon=floor(min_lon*100)/100;
    //    max_lon=floor(max_lon*100)/100;
    //    min_lat=floor(min_lat*100)/100;
    //    max_lat=floor(max_lat*100)/100;
    //    std::vector<unsigned int> out_ids;
    //    for(double lon=min_lon-0.01; lon<=max_lon+0.01; lon=lon+0.01){
    //        for(double lat=min_lat-0.01; lat<=max_lat+0.01; lat=lat+0.01){
    //            unsigned int block_id;
    //            Eigen::Vector3d gps_latlon;
    //            gps_latlon(0)=lon;
    //            gps_latlon(1)=lat;
    //            get_map_block_id_from_gps(block_id, gps_latlon);
    //            out_ids.push_back(block_id);
    //        }
    //    }
    //    return out_ids;        
    //}
    
    bool checkIDExist(std::vector<unsigned int>& input_ids, unsigned int query){
        for(int i=0; i<input_ids.size(); i++){
            if(input_ids[i]==query){
                return true;
            }
        }
        return false;
    }
    
    void load_global_map_by_gps(GlobalMap& map, std::string file_addr, Eigen::Vector3d gps_position){
        unsigned int block_id;
        get_map_block_id_from_gps(block_id, gps_position);
        std::vector<unsigned int> map_ids;
        map_ids.push_back(block_id);
        std::vector<unsigned int> near_blocks = getNearBlock(map_ids);
        load_global_map(map, file_addr, near_blocks);
    }
    
    void load_global_map(GlobalMap& map, std::string file_addr, std::vector<unsigned int> map_ids){
        for(int i=0; i<map_ids.size(); i++){
            unsigned int map_id_temp=map_ids[i];
            std::stringstream ss;
            ss<<map_id_temp;
            if(i==0){
                get_gps_from_block_id(map.gps_anchor, map_ids[i]);
            }
            GlobalMap map_temp;
            //std::cout<<"load_submap: "<<file_addr+"/"+ss.str()+".map"<<std::endl;
            if(load_submap(map_temp, file_addr+"/"+ss.str()+".map")){
                get_gps_from_block_id(map_temp.gps_anchor, map_ids[i]);
                for(int j=0; j<map_temp.frames.size(); j++){
                    Eigen::Vector3d out_tar_xyz;
                    convert_to_another_anchor(map_temp.gps_anchor, map.gps_anchor, map_temp.frames[j]->position, out_tar_xyz);
                    map_temp.frames[j]->position=out_tar_xyz;
                    convert_to_another_anchor(map_temp.gps_anchor, map.gps_anchor, map_temp.frames[j]->gps_position, out_tar_xyz);
                    map_temp.frames[j]->gps_position=out_tar_xyz;
                    map.frames.push_back(map_temp.frames[j]);
                }
                for(int j=0; j<map_temp.mappoints.size(); j++){
                    Eigen::Vector3d out_tar_xyz;
                    convert_to_another_anchor(map_temp.gps_anchor, map.gps_anchor, map_temp.mappoints[j]->position, out_tar_xyz);
                    map_temp.mappoints[j]->position=out_tar_xyz;
                    map.mappoints.push_back(map_temp.mappoints[j]);
                }
                for(int i=0; i<map_temp.pose_graph_v1.size(); i++){
                    map.pose_graph_v1.push_back(map_temp.pose_graph_v1[i]);
                    map.pose_graph_v2.push_back(map_temp.pose_graph_v2[i]);
                    map.pose_graph_e_posi.push_back(map_temp.pose_graph_e_posi[i]);
                    map.pose_graph_e_rot.push_back(map_temp.pose_graph_e_rot[i]);
                    map.pose_graph_e_scale.push_back(map_temp.pose_graph_e_scale[i]);
                    map.pose_graph_weight.push_back(map_temp.pose_graph_weight[i]);
                }
            }
        }
        std::unordered_map<long unsigned int, std::shared_ptr<MapPoint>> temp_mp_map;
        for(int i=0; i<map.mappoints.size(); i++){
            temp_mp_map[map.mappoints[i]->id]=map.mappoints[i];
        }
        for(int i=0; i<map.frames.size(); i++){
            if(map.frames[i]->imu_next_frame_id!=-1){
                map.frames[i]->imu_next_frame=map.getFrameById(map.frames[i]->imu_next_frame_id);
                if(map.frames[i]->imu_next_frame==nullptr){
                    map.frames[i]->isborder=true;
                }
            }
            for(int j=0; j<map.frames[i]->obss_ids.size(); j++){
                if(map.frames[i]->obss_ids[j]!= (long unsigned int)-1){
                    std::unordered_map<long unsigned int, std::shared_ptr<MapPoint>>::const_iterator it = temp_mp_map.find(map.frames[i]->obss_ids[j]);
                    if(it!=temp_mp_map.end()){
                        map.frames[i]->obss[j]=it->second;
                    }
                }
            }
        }
        
        GlobalMap nearmap=map;
//         std::vector<unsigned int> near_blocks = getNearBlock(map_ids);
//         for(int i=0; i<near_blocks.size(); i++){
//             unsigned int map_id_temp=near_blocks[i];
//             if(checkIDExist(map_ids, map_id_temp)){
//                 continue;
//             }
//             std::stringstream ss;
//             ss<<map_id_temp;
//             GlobalMap map_temp;
//             //std::cout<<"load_submap: "<<file_addr+"/"+ss.str()+".map"<<std::endl;
//             if(load_submap(map_temp, file_addr+"/"+ss.str()+".map")){
//                 for(int j=0; j<map_temp.frames.size(); j++){
//                     nearmap.frames.push_back(map_temp.frames[j]);
//                 }
//                 for(int j=0; j<map_temp.mappoints.size(); j++){
//                     nearmap.mappoints.push_back(map_temp.mappoints[j]);
//                 }
// 
//             }
//         }
//         temp_mp_map.clear();
//         for(int i=0; i<nearmap.mappoints.size(); i++){
//             temp_mp_map[nearmap.mappoints[i]->id]=nearmap.mappoints[i];
//         }
//         for(int i=0; i<nearmap.frames.size(); i++){
//             for(int j=0; j<nearmap.frames[i]->obss_ids.size(); j++){
//                 if(nearmap.frames[i]->obss_ids[j]!= (long unsigned int)-1){
//                     std::unordered_map<long unsigned int, std::shared_ptr<MapPoint>>::const_iterator it = temp_mp_map.find(nearmap.frames[i]->obss_ids[j]);
//                     if(it!=temp_mp_map.end()){
//                         nearmap.frames[i]->obss[j]=it->second;
//                     }
//                 }
//             }
//         }
//         nearmap.AssignKpToMp();
//         nearmap.CalConnections();
//         for(int i=0; i<nearmap.pose_graph_v1.size(); i++){
//             unsigned int block_id1;
//             get_map_block_id_from_id(block_id1, nearmap.pose_graph_v1[i]->id);
//             unsigned int block_id2;
//             get_map_block_id_from_id(block_id2, nearmap.pose_graph_v2[i]->id);
//             if(checkIDExist(map_ids, block_id1) && !checkIDExist(map_ids, block_id2)){
//                 //std::cout<<block_id1<<std::endl;
//                 nearmap.pose_graph_v1[i]->isborder=true;
//             }
//             if(!checkIDExist(map_ids, block_id1) && checkIDExist(map_ids, block_id2)){
//                 //std::cout<<block_id2<<std::endl;
//                 nearmap.pose_graph_v2[i]->isborder=true;
//             }
//         }
        map.AssignKpToMp();
    }
}   
