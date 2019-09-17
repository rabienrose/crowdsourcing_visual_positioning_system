#include <string>
#include <fstream>
#include <memory>
#include <glog/logging.h>
#include <gflags/gflags.h>
#include <backend/header.h>
#include "chamo_common/common.h"
#include "visualization/color-palette.h"
#include "visualization/color.h"
#include "visualization/common-rviz-visualization.h"

DEFINE_string(res_root, "", "");
DEFINE_string(out_root, "", "");
DEFINE_string(op_type, "", "");
DEFINE_string(param1, "", "");
DEFINE_string(map_ids, "", "");
DEFINE_string(project_mat_file, "", "");
DECLARE_string(reset_type);
DECLARE_bool(reset_val);

int main(int argc, char* argv[]) {
    google::InitGoogleLogging(argv[0]);
    google::InstallFailureSignalHandler();
    google::ParseCommandLineFlags(&argc, &argv, true);
    visualization::RVizVisualizationSink::init();
    
    std::string res_root=FLAGS_res_root;
    std::vector<std::string> ids_str=chamo::split(FLAGS_map_ids, ",");
    std::vector<unsigned int> map_ids;
    for(int i=0; i<ids_str.size(); i++){
        unsigned int map_id=std::stoul(ids_str[i]);
        map_ids.push_back(map_id);
    }
    gm::GlobalMap map;
    gm::load_global_map(map, res_root,map_ids);
    map.AssignKpToMp();
    std::vector<Eigen::Vector3d> debug_kf_posi;
    std::vector<Eigen::Vector3d> debug_mp_posi;
    std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> debug_matches;
    bool map_is_change; 
    bool match_is_change; 
    std::string status;
    std::cout<<"backend op: "<<FLAGS_op_type<<std::endl;
    if(FLAGS_op_type=="BA_rt"){
        optimize_BA(map, true, debug_kf_posi, debug_mp_posi, map_is_change, status);
    }else if(FLAGS_op_type=="BA"){
        optimize_BA(map, false,debug_kf_posi, debug_mp_posi, map_is_change, status);
    }else if(FLAGS_op_type=="pose_opt"){
        pose_graph_opti_se3(map);
    }else if(FLAGS_op_type=="Match"){
        update_corresponds(map, FLAGS_project_mat_file, debug_mp_posi, debug_kf_posi, debug_matches, map_is_change, match_is_change);
    }else if(FLAGS_op_type=="CullingFrame"){
        culling_frame(map);
    }else if(FLAGS_op_type=="Reset"){
        reset_all_status(map, FLAGS_reset_type, FLAGS_reset_val);
    }else if(FLAGS_op_type=="sim3"){
        pose_graph_opti_sim3(map);
    }else if(FLAGS_op_type=="test"){
        std::vector<std::vector<std::shared_ptr<gm::Frame>>> group_frames;
        cal_subgroup(map, group_frames);
    }
    gm::save_global_map(map, FLAGS_out_root);
}