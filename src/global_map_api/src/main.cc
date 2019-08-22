#include "global_map_api/global_map_api.h"
#include <glog/logging.h>
#include <gflags/gflags.h>
#include "ros/ros.h"
#include "visualization/color-palette.h"
#include "visualization/color.h"
#include "visualization/common-rviz-visualization.h"

DEFINE_string(map_root, "", "");
DEFINE_string(cache_root, "", "");
DEFINE_string(localmap_root, "", "");
DEFINE_string(bag_addr, "", "");
DEFINE_string(config_root, "", "");
DEFINE_string(image_addr, "", "");

int main(int argc, char* argv[]) {
    google::InitGoogleLogging(argv[0]);
    google::InstallFailureSignalHandler();
    google::ParseCommandLineFlags(&argc, &argv, true);
    ros::init(argc, argv, "vis_loc");
    ros::NodeHandle nh;
    visualization::RVizVisualizationSink::init();
    gm::GlobalMapApi api;
    api.init(FLAGS_config_root, FLAGS_map_root);
    std::string status;
    api.process_bag(FLAGS_bag_addr, FLAGS_cache_root, FLAGS_localmap_root, status);
    return 0;
}