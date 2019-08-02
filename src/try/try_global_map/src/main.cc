#include "visual_map/visual_map.h"
#include "visual_map/visual_map_seri.h"
#include "global_map/global_map.h"
#include "global_map/global_map_seri.h"
#include <glog/logging.h>
#include <gflags/gflags.h>

DEFINE_string(res_root, "", "");
  
int main(int argc, char* argv[]){
    
    google::InitGoogleLogging(argv[0]);
    google::InstallFailureSignalHandler();
    google::ParseCommandLineFlags(&argc, &argv, true);
    std::string res_root=FLAGS_res_root;

    vm::VisualMap map;
    vm::loader_visual_map(map, res_root);
    
    

    return 0;
}