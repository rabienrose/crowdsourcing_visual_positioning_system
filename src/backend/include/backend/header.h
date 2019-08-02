
#include <string>
#include <fstream>
#include <memory>
#include <glog/logging.h>
#include <gflags/gflags.h>
#include <global_match/global_match.h>
#include <global_map/global_map_seri.h>
#include <global_map/global_map.h>
#include <sim3_ransac/sim3_match.h>
void update_corresponds(std::shared_ptr<gm::GlobalMap> map_p);
void pose_graph_opti(std::shared_ptr<gm::GlobalMap> map_p);