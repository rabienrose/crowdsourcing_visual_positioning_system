#include <string>
#include <fstream>
#include <memory>
#include <Eigen/Core>
#include <math.h>
#include <unordered_map>

#include <backend/header.h>
#include "opencv2/opencv.hpp"
#include <glog/logging.h>
#include <gflags/gflags.h>

void getAllChildren(gm::GlobalMap& map, std::set<std::shared_ptr<gm::Frame>>& children, std::shared_ptr<gm::Frame> frame_p){
    std::vector<std::shared_ptr<gm::Frame>> next_level_frames_temp;
    map.getFrameChildren(next_level_frames_temp, frame_p);
    for(int i=0; i<next_level_frames_temp.size(); i++){
        auto search = children.find(next_level_frames_temp[i]);
        if (search == children.end()) {
            children.insert(next_level_frames_temp[i]);
            getAllChildren(map, children, next_level_frames_temp[i]);
        }
    }
}

void cal_subgroup(gm::GlobalMap& map, std::vector<std::vector<std::shared_ptr<gm::Frame>>>& group_frames){
    map.CalConnections();
    map.FilterTrack();
    group_frames.clear();
    std::vector<std::vector<std::shared_ptr<gm::Frame>>> group_frames_temp;
    std::set<std::shared_ptr<gm::Frame>> processed;
    for(int i=0; i<map.frames.size(); i++){
        auto search = processed.find(map.frames[i]);
        if (search == processed.end()) {
            std::set<std::shared_ptr<gm::Frame>> children;
            getAllChildren(map, children, map.frames[i]);
            
            std::vector<std::shared_ptr<gm::Frame>> frames;
            for(auto f : children) {
                processed.insert(f);
                frames.push_back(f);
            } 
            group_frames_temp.push_back(frames);
        }
    }
    for(int i=0;i<group_frames_temp.size(); i++){
        if(group_frames_temp[i].size()>20){
            group_frames.push_back(group_frames_temp[i]);
        }
    }
}
