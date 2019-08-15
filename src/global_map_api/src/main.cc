#include "global_map_api/global_map_api.h"
#include <glog/logging.h>
#include <gflags/gflags.h>

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
    gm::GlobalMapApi api;
    api.init(FLAGS_config_root, FLAGS_map_root);
    api.process_bag(FLAGS_bag_addr, FLAGS_cache_root, FLAGS_localmap_root);
    return 0;
}