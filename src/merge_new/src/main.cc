#include <glog/logging.h>
#include <gflags/gflags.h>
#include "chamo_common/common.h"
#include "merge_new/merge_new.h"
DEFINE_string(base_map_root, "", "");
DEFINE_string(new_map_root, "", "");
DEFINE_string(out_map_root, "", "");
DEFINE_string(map_ids, "", "");

int main(int argc, char* argv[]) {
    google::InitGoogleLogging(argv[0]);
    google::InstallFailureSignalHandler();
    google::ParseCommandLineFlags(&argc, &argv, true);
    merge_new(FLAGS_base_map_root, FLAGS_new_map_root, FLAGS_out_map_root, chamo::split(FLAGS_map_ids,","));
    return 0;
}