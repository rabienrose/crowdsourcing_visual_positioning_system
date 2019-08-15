#include "convert_to_visual_map/convert_to_visual_map.h"
#include <glog/logging.h>
#include <gflags/gflags.h>
DEFINE_string(res_root, "", "");
DEFINE_string(global_root, "", "");
int main(int argc, char* argv[]) {
    google::InitGoogleLogging(argv[0]);
    google::InstallFailureSignalHandler();
    google::ParseCommandLineFlags(&argc, &argv, true);
    std::string res_root=FLAGS_res_root;
    std::vector<unsigned int> ids;
    convert_to_visual_map(res_root, FLAGS_global_root, ids);
    return 0;
}