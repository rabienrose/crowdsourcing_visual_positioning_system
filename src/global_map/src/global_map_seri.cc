#include "global_map/global_map_seri.h"
#include <glog/logging.h>

#define ID_LEN 1000000000

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
    void putToFile(long unsigned int val, std::fstream& f){
        f.write((char*)&val, 8);
    }
    long unsigned int getFromFileLI(std::fstream& f){
        long unsigned int val;
        f.read((char*)&val, 8);
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
    
    unsigned int getMapBlockId(long unsigned int whole_id){
        unsigned int block_id = whole_id>>ID_LEN;
        return block_id;
    }

    void save_submap(GlobalMap& map, std::string file_addr){
        std::fstream output(file_addr, std::ios::out | std::ios::trunc | std::ios::binary);
        putToFile((int)map.mappoints.size(), output);
        std::cout<<"mp count: "<<map.mappoints.size()<<std::endl;
        for(size_t i=0; i<map.mappoints.size(); i++){
            std::shared_ptr<MapPoint> mappoint_p = map.mappoints[i];
            putToFile(mappoint_p->id, output);
            putToFileD2F(mappoint_p->position.x(), output);
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
                if(frame_p->obss[j]!=nullptr){
                    putToFile(frame_p->obss[j]->id, output);
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
        output.close();
    }

    bool loader_submap(GlobalMap& map, std::string file_addr){
        std::fstream input(file_addr.c_str(), std::ios::in | std::ios::binary);
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
            //std::cout<<mappoint_p->match_count<<std::endl;
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
        return true;
    }
    
    void save_global_map(GlobalMap& map, std::string file_addr){
        std::map<unsigned int, std::shared_ptr<GlobalMap>> submaps;
        for(int i=0; i<map.frames.size(); i++){
            unsigned int block_id = getMapBlockId(map.frames[i]->id);
            if(submaps.count(block_id)==0){
                submaps[block_id].reset(new GlobalMap);
            }
            submaps[block_id]->frames.push_back(map.frames[i]);
        }
        for(int i=0; i<map.mappoints.size(); i++){
            unsigned int block_id = getMapBlockId(map.mappoints[i]->id);
            if(submaps.count(block_id)==0){
                submaps[block_id].reset(new GlobalMap);
            }
            submaps[block_id]->mappoints.push_back(map.mappoints[i]);
        }
        for(std::map<unsigned int, std::shared_ptr<GlobalMap>>::iterator it=submaps.begin(); it!=submaps.end(); it++){
            std::stringstream ss;
            ss<<it->first;
            save_submap(*(it->second), file_addr+"/"+ss.str()+".map");
        }
    }
    
    void load_global_map(GlobalMap& map, std::string file_addr, std::vector<unsigned int> map_ids){
        for(int i=0; i<map_ids.size(); i++){
            unsigned int map_id_temp=map_ids[i];
            std::stringstream ss;
            ss<<map_id_temp;
            GlobalMap map_temp;
            if(loader_submap(map_temp, file_addr+"/"+ss.str()+".map")){
                map.frames.insert(map.frames.end(), map_temp.frames.begin(), map_temp.frames.end());
                map.mappoints.insert(map.mappoints.end(), map_temp.mappoints.begin(), map_temp.mappoints.end());
            }
        }
        for(int i=0; i<map.frames.size(); i++){
            if(map.frames[i]->imu_next_frame_id!=-1){
                map.frames[i]->imu_next_frame=map.getFrameById(map.frames[i]->imu_next_frame_id);
            }
            for(int j=0; j<map.frames[i]->obss_ids.size(); j++){
                if(map.frames[i]->obss_ids[j]!= -1){
                    map.frames[i]->obss[j]=map.getMPById(map.frames[i]->obss_ids[j]);
                }
            }
        }
        map.AssignKpToMp();
    }
}   