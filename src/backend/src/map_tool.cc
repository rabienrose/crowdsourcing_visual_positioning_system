#include <iostream>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <backend/header.h>

DEFINE_string(reset_type, "all", "");
DEFINE_bool(reset_val, false, "");

void reset_all_status(gm::GlobalMap& map){
    for(int i=0; i<map.frames.size(); i++){
        if(FLAGS_reset_type=="all"){
            map.frames[i]->isfix=FLAGS_reset_val;
            map.frames[i]->doBA=FLAGS_reset_val;
            map.frames[i]->doGraphOpti=FLAGS_reset_val;
            map.frames[i]->doMatch=FLAGS_reset_val;
        }
        
    }
    for(int i=0; i<map.mappoints.size(); i++){
        if(FLAGS_reset_type=="all"){
            map.mappoints[i]->isfix=FLAGS_reset_val;
            map.mappoints[i]->isbad=FLAGS_reset_val;
        }
    }
}
        