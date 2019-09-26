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
    std::string final_addr=FLAGS_workspace_root+"/final";
    if(fs::exists(reject_addr)){
        fs::remove_all(reject_addr);
    }
    fs::create_directories(reject_addr);
    if(!fs::exists(global_addr)){
        fs::create_directories(global_addr);
    }
    if(!fs::exists(final_addr)){
        fs::create_directories(final_addr);
    }
    api.init(FLAGS_workspace_root+"/config",global_addr);
    std::vector<std::string> ranked_files;
    std::map<std::string, float> file_sizes;
    for(auto& p: fs::directory_iterator(FLAGS_bag_addr)){
        std::vector<std::string> splited= chamo::split(p.path().string(), ".");
        if(splited.back()!="bag"){
            continue;
        }
        file_sizes[p.path().string()]=fs::file_size(p);
    }
    typedef std::function<bool(std::pair<std::string, float>, std::pair<std::string, float>)> Comparator;
    Comparator compFunctor =
            [](std::pair<std::string, float> elem1 ,std::pair<std::string, float> elem2)
            {
                return elem1.second > elem2.second;
            };
    std::set<std::pair<std::string, float>, Comparator> setOfWords(
            file_sizes.begin(), file_sizes.end(), compFunctor);
    for (std::pair<std::string, float> element : setOfWords){
        ranked_files.push_back(element.first);
    }
    for(auto& p: ranked_files){
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
        std::cout<<"process bag: "<<p<<std::endl;
        std::string status;
        if(api.process_bag(p, cache_addr, local_addr, status)){
            if(fs::exists(release_addr)){
                fs::remove_all(release_addr);
            }
            fs::create_directories(release_addr);
            fs::copy(global_addr, release_addr, std::filesystem::copy_options::recursive);
        }else{
            fs::copy(p, reject_addr);
        }
    }
    std::vector<unsigned int> block_ids;
    for(auto& p: fs::directory_iterator(release_addr)){
        std::vector<std::string> splited= chamo::split(p.path().string(), ".");
        if(splited.size()==2){
            if(splited.back()!="map"){
                continue;
            }
            std::vector<std::string> splited1= chamo::split(splited[0], "/");
            
            unsigned int map_id=std::stoul(splited1.back());
            block_ids.push_back(map_id);
        }
    }
    api.final_proc(release_addr, final_addr, block_ids);
    
    
    return 0;
}