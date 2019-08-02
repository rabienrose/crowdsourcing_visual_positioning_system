#include <string>
#include <fstream>
#include <memory>
#include <glog/logging.h>
#include <gflags/gflags.h>
#include <backend/header.h>
DEFINE_string(map_addr, "", "Folder of the map file, also the place to save the new map file.");
DEFINE_string(map_name, "", "File name of map file.");

int main(int argc, char* argv[]) {
    google::InitGoogleLogging(argv[0]);
    google::InstallFailureSignalHandler();
    google::ParseCommandLineFlags(&argc, &argv, true);
    
    std::shared_ptr<gm::GlobalMap> map_p;
    map_p.reset(new gm::GlobalMap);
    gm::GlobalMap& map=*map_p;
    std::cout<<"pose graph: "<<FLAGS_map_addr+"/"+FLAGS_map_name<<std::endl;
    gm::load_submap(map, FLAGS_map_addr+"/"+FLAGS_map_name);
    map_p->AssignKpToMp();
    map_p->CalConnections();
    //update_corresponds(map_p);
    pose_graph_opti(map_p);
    gm::save_submap(map, FLAGS_map_addr+"/pose_"+FLAGS_map_name);
}