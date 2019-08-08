#include <string>
#include <fstream>
#include <memory>
#include <glog/logging.h>
#include <gflags/gflags.h>
#include <backend/header.h>
#include "chamo_common/common.h"
#include "visualization/color-palette.h"
#include "visualization/color.h"
#include "visualization/common-rviz-visualization.h"

DEFINE_string(res_root, "", "");
DEFINE_string(out_root, "", "");
DEFINE_string(map_ids, "", "");

int main(int argc, char* argv[]) {
    google::InitGoogleLogging(argv[0]);
    google::InstallFailureSignalHandler();
    google::ParseCommandLineFlags(&argc, &argv, true);
    visualization::RVizVisualizationSink::init();
    
    std::string res_root=FLAGS_res_root;
    std::vector<std::string> ids_str=chamo::split(FLAGS_map_ids, ",");
    std::vector<unsigned int> map_ids;
    for(int i=0; i<ids_str.size(); i++){
        unsigned int map_id=std::stoul(ids_str[i]);
        map_ids.push_back(map_id);
    }
    gm::GlobalMap map;
    gm::load_global_map(map, res_root,map_ids);
    map.AssignKpToMp();
    map.CalConnections();
    std::cout<<"connections: "<<map.pose_graph_v1.size()<<std::endl;
    update_corresponds(map);
    pose_graph_opti(map);
    gm::save_global_map(map, FLAGS_out_root);
}