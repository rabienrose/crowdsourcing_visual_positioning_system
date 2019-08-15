#include <string>
#include <fstream>
#include <memory>
#include <opencv2/opencv.hpp>
#include <Eigen/Core>
#include "global_map/global_map.h"
#include "global_map/global_map_seri.h"
#include "chamo_common/common.h"
#include <glog/logging.h>
#include <gflags/gflags.h>
#include "merge_new/merge_new.h"

void merge_new(std::string base_map_root, std::string new_map_root, std::string out_map_root, std::vector<unsigned int> ids) {
    gm::GlobalMap base_map;
    gm::GlobalMap new_map;
    gm::load_global_map(base_map, base_map_root, ids);
    gm::load_global_map(new_map, new_map_root, ids);
    if(base_map.frames.size()==0 && base_map.mappoints.size()==0){
        base_map=new_map;
        for(int i=0; i<base_map.frames.size(); i++){
            new_map.frames[i]->doMatch=true;
        }
    }else{
        for(int i=0; i<new_map.frames.size(); i++){
            new_map.frames[i]->doMatch=true;
            base_map.frames.push_back(new_map.frames[i]);
        }
        for(int i=0; i<new_map.mappoints.size(); i++){
            base_map.mappoints.push_back(new_map.mappoints[i]);
        }
    }
    
    gm::save_global_map(base_map, out_map_root);
}