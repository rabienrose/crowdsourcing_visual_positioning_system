#include "global_map_api/global_map_api.h"
#include "chamo_common/common.h"
#include <glog/logging.h>
#include <gflags/gflags.h>
#include "ros/ros.h"
#include "visualization/color-palette.h"
#include "visualization/color.h"
#include "visualization/common-rviz-visualization.h"
#include <filesystem>

DEFINE_string(workspace_root, "", "");
DEFINE_string(bag_addr, "", "");
namespace fs = std::filesystem;
int main(int argc, char* argv[]) {
    google::InitGoogleLogging(argv[0]);
    google::InstallFailureSignalHandler();
    google::ParseCommandLineFlags(&argc, &argv, true);
    ros::init(argc, argv, "vis_loc");
    ros::NodeHandle nh;
    visualization::RVizVisualizationSink::init();
    gm::GlobalMapApi api;
    std::string global_addr=FLAGS_workspace_root+"/global";
    std::string release_addr=FLAGS_workspace_root+"/release";
    std::string local_addr=FLAGS_workspace_root+"/local";
    std::string cache_addr=FLAGS_workspace_root+"/cache";
    std::string reject_addr=FLAGS_workspace_root+"/reject";
    if(fs::exists(reject_addr)){
        fs::remove_all(reject_addr);
    }
    fs::create_directories(reject_addr);
    if(!fs::exists(global_addr)){
        fs::create_directories(FLAGS_workspace_root+"/global");
    }
    api.init(FLAGS_workspace_root+"/config",global_addr);
    for(auto& p: fs::directory_iterator(FLAGS_bag_addr)){
        std::vector<std::string> splited= chamo::split(p.path().string(), ".");
        if(splited.back()!="bag"){
            continue;
        }
        if(fs::exists(local_addr)){
            fs::remove_all(local_addr);
        }
        fs::create_directories(local_addr);
        if(fs::exists(cache_addr)){
            fs::remove_all(cache_addr);
        }
        fs::create_directories(cache_addr);
        if(fs::exists(global_addr)){
            fs::remove_all(global_addr);
        }
        fs::create_directories(global_addr);
        if(fs::exists(release_addr)){
            fs::copy(release_addr, global_addr, std::filesystem::copy_options::recursive);
        }else{
            fs::create_directories(release_addr);
        }
        std::cout<<"process bag: "<<p.path().string()<<std::endl;
        std::string status;
        if(api.process_bag(p.path().string(), cache_addr, local_addr, status)){
            if(fs::exists(release_addr)){
                fs::remove_all(release_addr);
            }
            fs::create_directories(release_addr);
            fs::copy(global_addr, release_addr, std::filesystem::copy_options::recursive);
        }else{
            fs::copy(p.path().string(), reject_addr);
        }
    }
    
    return 0;
}