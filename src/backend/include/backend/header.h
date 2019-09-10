
#include <string>
#include <fstream>
#include <memory>
#include <glog/logging.h>
#include <gflags/gflags.h>
#include <global_match/global_match.h>
#include <global_map/global_map_seri.h>
#include <global_map/global_map.h>
#include <sim3_ransac/sim3_match.h>
#include <unordered_map>
bool update_corresponds(gm::GlobalMap& map, std::string project_mat_file,std::vector<Eigen::Vector3d>& debug_mp_posi,
                        std::vector<Eigen::Vector3d>& debug_kf_posi, std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>>& debug_matches,
                        bool& map_is_change, bool& match_is_change);
void pose_graph_opti_sim3(gm::GlobalMap& map);
void pose_graph_opti_se3(gm::GlobalMap& map);
void optimize_BA(gm::GlobalMap& map, bool re_triangle, std::vector<Eigen::Vector3d>& debug_kf_posi, std::vector<Eigen::Vector3d>& debug_mp_posi, bool& map_is_change, std::string& status);
void culling_frame(gm::GlobalMap& map);
void reset_all_status(gm::GlobalMap& map, std::string reset_type, bool reset_val);
void cal_subgroup(gm::GlobalMap& map, std::vector<std::vector<std::shared_ptr<gm::Frame>>>& ranked_group_frames);
void cal_subgroup_remove(gm::GlobalMap& map, int N, std::vector<std::vector<std::shared_ptr<gm::Frame>>>& ranked_group_frames);
