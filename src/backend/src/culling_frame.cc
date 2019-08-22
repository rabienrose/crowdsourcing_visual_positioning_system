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
                            double c1=it->first->gps_accu;
                            double c2=map.frames[i]->gps_accu;
                            Eigen::Vector3d p1=it->first->gps_position+(map.frames[i]->position-it->first->position);
                            Eigen::Vector3d p2=map.frames[i]->gps_position;
                            Eigen::Vector3d merged_gps=(c1*c1*p2+c2*c2*p1)/(c1*c1+c2*c2);
                            double merged_cov=sqrt(1/(1/(c1*c1)+1/(c2*c2)));
                            std::cout<<c1<<":"<<c2<<":"<<merged_cov<<std::endl;
                            std::cout<<p2.transpose()<<std::endl;
                            std::cout<<merged_gps.transpose()<<std::endl;
                            map.frames[i]->gps_position=merged_gps;
                            map.frames[i]->gps_accu=merged_cov;
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
    }while(del_any_frame==true);
}
        