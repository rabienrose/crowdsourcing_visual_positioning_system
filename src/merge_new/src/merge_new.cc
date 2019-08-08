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

DEFINE_string(base_map_root, "", "");
DEFINE_string(new_map_root, "", "");
DEFINE_string(out_map_root, "", "");
DEFINE_string(map_ids, "", "");

int main(int argc, char* argv[]) {
    google::InitGoogleLogging(argv[0]);
    google::InstallFailureSignalHandler();
    google::ParseCommandLineFlags(&argc, &argv, true);
    
    std::string base_map_root=FLAGS_base_map_root;
    std::string new_map_root=FLAGS_new_map_root;
    std::vector<std::string> ids_str=chamo::split(FLAGS_map_ids, ",");
    std::vector<unsigned int> map_ids;
    for(int i=0; i<ids_str.size(); i++){
        unsigned int map_id=std::stoul(ids_str[i]);
        map_ids.push_back(map_id);
    }
    gm::GlobalMap base_map;
    gm::GlobalMap new_map;
    gm::load_global_map(base_map, base_map_root, map_ids);
    gm::load_global_map(new_map, new_map_root, map_ids);
    for(int i=0; i<new_map.frames.size(); i++){
        new_map.frames[i]->doMatch=true;
        base_map.frames.push_back(new_map.frames[i]);
    }
    for(int i=0; i<new_map.mappoints.size(); i++){
        base_map.mappoints.push_back(new_map.mappoints[i]);
    }
    
    gm::save_global_map(base_map, FLAGS_out_map_root);
    
    return 0;
}