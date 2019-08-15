#include "global_map_api/global_map_api.h"
#include <glog/logging.h>
#include <gflags/gflags.h>

DEFINE_string(map_root, "", "");
DEFINE_string(cache_root, "", "");
DEFINE_string(bag_addr, "", "");
DEFINE_string(config_root, "", "");
DEFINE_string(image_addr, "", "");
DEFINE_string(op_type, "", "");
int main(int argc, char* argv[]) {
    google::InitGoogleLogging(argv[0]);
    google::InstallFailureSignalHandler();
    google::ParseCommandLineFlags(&argc, &argv, true);
    GlobalMapApi api;
    api.init(FLAGS_config_root, FLAGS_map_root);
    if(FLAGS_op_type=="map"){
        api.process_bag(FLAGS_bag_addr, FLAGS_cache_root);
    }else if(FLAGS_op_type=="loc_img"){
        api.load_map(Eigen::Vector3d gps_position);
        Eigen::Matrix4d pose
        api.img(Eigen::Vector3d gps_position);
    }
    return 0;
}