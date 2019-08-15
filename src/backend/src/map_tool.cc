#include <iostream>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <backend/header.h>

DEFINE_string(reset_type, "all", "");
DEFINE_bool(reset_val, false, "");

void reset_all_status(gm::GlobalMap& map, std::string reset_type, bool reset_val){
    for(int i=0; i<map.frames.size(); i++){
        if(reset_type=="all"){
            map.frames[i]->isfix=reset_val;
            map.frames[i]->doBA=reset_val;
            map.frames[i]->doGraphOpti=reset_val;
            map.frames[i]->doMatch=reset_val;
        }
        if(reset_type=="doMatch"){
            map.frames[i]->doMatch=reset_val;
        }
        if(reset_type=="isfix"){
            map.frames[i]->isfix=reset_val;
        }
        
    }
    for(int i=0; i<map.mappoints.size(); i++){
        if(reset_type=="all"){
            map.mappoints[i]->isfix=reset_val;
            map.mappoints[i]->isbad=reset_val;
        }
        if(reset_type=="isfix"){
            map.mappoints[i]->isfix=reset_val;
        }
    }
}
        