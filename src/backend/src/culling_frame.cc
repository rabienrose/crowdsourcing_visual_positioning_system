#include <iostream>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <backend/header.h>

DEFINE_double(cull_frame_rate, 0.8, "");

void culling_frame(gm::GlobalMap& map){
    map.FilterTrack();
    bool del_any_frame=false;
    std::unordered_map<long unsigned int, int> frame_obss_count;
    for(int i=0; i<map.frames.size(); i++){
        int cont_t=0;
        for(int j=0; j<map.frames[i]->obss.size(); j++){
            if(map.frames[i]->obss[j]!=nullptr){
                cont_t++;
            }
        }
        frame_obss_count[map.frames[i]->id]=cont_t;
    }
    do{
        std::cout<<map.frames.size()<<std::endl;
        del_any_frame=false;
        for(int i=0; i<map.frames.size(); i++){
            std::map<std::shared_ptr<gm::Frame>, int> frame_list;
            map.GetCovisi(map.frames[i], frame_list);
            std::map<std::shared_ptr<gm::Frame>, int>::iterator it;
            for ( it = frame_list.begin(); it != frame_list.end(); it++ ){
                CHECK_NE(map.frames[i]->id, -1);
                CHECK_NE(it->first->id, -1);
                float rate_1=it->second/(float)frame_obss_count[map.frames[i]->id];
                float rate_2=it->second/(float)frame_obss_count[it->first->id];
                
                if(rate_1>FLAGS_cull_frame_rate){
                    if(rate_2>FLAGS_cull_frame_rate){
                        //std::cout<<rate_1<<":"<<rate_2<<std::endl;
                        //std::cout<<"del :"<<it->first->id<<std::endl;
                        //std::cout<<"it->second :"<<it->second<<std::endl;
                        int cont_t=0;
                        for(int j=0; j<map.frames[i]->obss.size(); j++){
                            if(map.frames[i]->obss[j]!=nullptr){
                                cont_t++;
                            }
                        }
                        //std::cout<<"cont_t :"<<cont_t<<std::endl;
                        if(it->first->isborder==false){
                            if(map.DelFrame(it->first->id)){
                                del_any_frame=true;
                                break;
                            }
                            
                        }
                    }
                }
            }
            if(del_any_frame){
                break;
            }
        }
        std::cout<<del_any_frame<<std::endl;
    }while(del_any_frame==true);
}
        