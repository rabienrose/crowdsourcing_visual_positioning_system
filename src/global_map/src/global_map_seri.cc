#include "global_map/global_map_seri.h"
#include <glog/logging.h>

namespace gm{
    void putToFile(float val, std::fstream& f){
        f.write((char*)&val, 4);
    }
    float getFromFileF(std::fstream& f){
        float val;
        f.read((char*)&val, 4);
        return val;
    }
    void putToFile(double val, std::fstream& f){
        f.write((char*)&val, 8);
    }
    double getFromFileD(std::fstream& f){
        double val;
        f.read((char*)&val, 8);
        return val;
    }
    void putToFileD2F(float val, std::fstream& f){
        float val_f=(float)val;
        f.write((char*)&val_f, 4);
    }
    double getFromFileF2D(std::fstream& f){
        float val_f;
        f.read((char*)&val_f, 4);
        return val_f;
    }
    void putToFile(int val, std::fstream& f){
        f.write((char*)&val, 4);
    }
    int getFromFileI(std::fstream& f){
        int val;
        f.read((char*)&val, 4);
        return val;
    }
    void putToFile(std::string& str, std::fstream& f){
        int str_len=str.size();
        f.write((char*)&str_len, 4);
        f.write((const char*)str.c_str(), str.size());
    }
    std::string getFromFileS(std::fstream& f){
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

    void save_global_map(GlobalMap& map, std::string file_addr){
        map.ComputeUniqueId();
        std::fstream output(file_addr, std::ios::out | std::ios::trunc | std::ios::binary);
        putToFile(map.gps_anchor.x(), output);
        putToFile(map.gps_anchor.y(), output);
        putToFile(map.gps_anchor.z(), output);
        putToFileD2F(map.Tbc_posi.x(), output);
        putToFileD2F(map.Tbc_posi.y(), output);
        putToFileD2F(map.Tbc_posi.z(), output);
        putToFileD2F(map.Tbc_qua.w(), output);
        putToFileD2F(map.Tbc_qua.x(), output);
        putToFileD2F(map.Tbc_qua.y(), output);
        putToFileD2F(map.Tbc_qua.z(), output);

        std::map<MapPoint*, int> mappoint_to_index;
        putToFile((int)map.mappoints.size(), output);
        std::cout<<"mp count: "<<map.mappoints.size()<<std::endl;
        for(size_t i=0; i<map.mappoints.size(); i++){
            std::shared_ptr<MapPoint> mappoint_p = map.mappoints[i];
            mappoint_to_index[mappoint_p.get()]=i;
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
            putToFile(frame_p->frame_file_name, output);
            putToFile(frame_p->time_stamp, output);
            
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

            std::vector<cv::KeyPoint>& keypoints1=frame_p->kps;
            putToFile((int)keypoints1.size(), output);
            for (size_t j=0; j<keypoints1.size(); j++){
                kp_count++;
                putToFile(keypoints1[j].pt.x, output);
                putToFile(keypoints1[j].pt.y, output);
                if(frame_p->obss[j]!=nullptr){
                    if(mappoint_to_index.count(frame_p->obss[j].get())==0){
                        putToFile(-1, output);
                        std::cout<<"[error]mappoint_to_index.count(frame_p->obss[j].get())==0  "<<frame_p->obss[j]->id<<std::endl;
                        //exit(0);
                    }else{
                        putToFile(mappoint_to_index[frame_p->obss[j].get()], output);
                    }
                }else{
                    putToFile(-1, output);
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
            if(frame_p->imu_next_frame==nullptr){
                putToFile((int)-1, output);
            }else{
                putToFile(frame_p->imu_next_frame->id, output);
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
    }

    bool loader_global_map(GlobalMap& map, std::string file_addr){
        std::fstream input(file_addr.c_str(), std::ios::in | std::ios::binary);
        if(!input.is_open()){
            return false;
        }
        map.gps_anchor(0) = getFromFileD(input);
        map.gps_anchor(1) = getFromFileD(input);
        map.gps_anchor(2) = getFromFileD(input);
        map.Tbc_posi.x()=getFromFileF2D(input);
        map.Tbc_posi.y()=getFromFileF2D(input);
        map.Tbc_posi.z()=getFromFileF2D(input);
        map.Tbc_qua.w()=getFromFileF2D(input);
        map.Tbc_qua.x()=getFromFileF2D(input);
        map.Tbc_qua.y()=getFromFileF2D(input);
        map.Tbc_qua.z()=getFromFileF2D(input);
        
        int mappoints_size=getFromFileI(input);
        std::cout<<"mp count: "<<mappoints_size<<std::endl;
        for(int i=0; i<mappoints_size; i++){
            std::shared_ptr< MapPoint> mappoint_p;
            mappoint_p.reset(new  MapPoint);
            mappoint_p->position.x()=getFromFileF2D(input);
            mappoint_p->position.y()=getFromFileF2D(input);
            mappoint_p->position.z()=getFromFileF2D(input);
            //mappoint_p->match_count=getFromFileI(input);
            //std::cout<<mappoint_p->match_count<<std::endl;
            map.mappoints.push_back(mappoint_p);
        }
        
        int frames_size;
        frames_size = getFromFileI( input);
        std::cout<<"frame count: "<<frames_size<<std::endl;
        int kp_count=0;
        std::vector<int> nextimu_frameids;
        for(int i=0; i<frames_size; i++){
            std::shared_ptr< Frame> frame_p;
            frame_p.reset(new  Frame);
            frame_p->frame_file_name = getFromFileS(input);
            frame_p->time_stamp=getFromFileD(input);
            
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
            
            int kps_size;
            kps_size=getFromFileI(input);
            frame_p->obss.resize(kps_size);
            for(int j=0; j<kps_size; j++){
                kp_count++;
                cv::KeyPoint kp;
                kp.pt.x=getFromFileF(input);
                kp.pt.y=getFromFileF(input);
                int obs_id=getFromFileI(input);
                if(obs_id==-1){
                    frame_p->obss[j]=nullptr;
                }else{
                    CHECK_GT(map.mappoints.size(), obs_id);
                    frame_p->obss[j]=map.mappoints[obs_id];
                }
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
            int next_imu_id=getFromFileI(input);
            nextimu_frameids.push_back(next_imu_id);
            map.frames.push_back(frame_p);
        }
        CHECK_EQ(nextimu_frameids.size(), map.frames.size());
        for(size_t i=0; i<map.frames.size(); i++){
            if(nextimu_frameids[i]!=-1){
                CHECK_GT(map.frames.size(),nextimu_frameids[i]);
                map.frames[i]->imu_next_frame=map.frames[nextimu_frameids[i]];
            }
        }
        std::cout<<"kp count: "<<kp_count<<std::endl;

        int pose_graph_e_size;
        pose_graph_e_size=getFromFileI(input);
        std::cout<<"covisi count: "<<pose_graph_e_size<<std::endl;
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
            map.pose_graph_e_posi.push_back(posi);
            Eigen::Matrix3d rot(rot_qua);
            map.pose_graph_e_rot.push_back(rot);
            map.pose_graph_e_scale.push_back(getFromFileF2D(input));
            map.pose_graph_weight.push_back(getFromFileF2D(input));
            
            int v1_id =getFromFileI(input);
            int v2_id =getFromFileI(input);
            CHECK_GT(map.frames.size(),v1_id);
            CHECK_GT(map.frames.size(),v2_id);
            map.pose_graph_v1.push_back(map.frames[v1_id]);
            map.pose_graph_v2.push_back(map.frames[v2_id]);
        }
        map.AssignKpToMp();
        return true;
    }
}   