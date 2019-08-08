
#include <string>
#include <fstream>
#include <memory>
#include <glog/logging.h>
#include <gflags/gflags.h>
#include <global_match/global_match.h>
#include <global_map/global_map_seri.h>
#include <global_map/global_map.h>
#include <sim3_ransac/sim3_match.h>
void update_corresponds(gm::GlobalMap& map);
void pose_graph_opti(gm::GlobalMap& map);