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

void cal_subgroup(gm::GlobalMap& map, std::vector<std::vector<std::shared_ptr<gm::Frame>>>& ranked_group_frames){
    map.CalConnections();
    map.FilterTrack();
    std::vector<std::vector<std::shared_ptr<gm::Frame>>> group_frames;
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
    for(int i=0; i<group_frames.size(); i++){
        std::cout<<group_frames[i].size()<<std::endl;
    }
    std::map<int, int> groupid_to_sizes;
    for(int i=0; i<group_frames.size(); i++){
        groupid_to_sizes[i]=group_frames[i].size();
    }
    typedef std::function<bool(std::pair<int, int>, std::pair<int, int>)> Comparator;
    Comparator compFunctor =
            [](std::pair<int, int> elem1 ,std::pair<int, int> elem2)
            {
                return elem1.second > elem2.second;
            };
    std::set<std::pair<int, int>, Comparator> setOfWords(
            groupid_to_sizes.begin(), groupid_to_sizes.end(), compFunctor);
    std::vector<int> ranked_groupid;
    for (std::pair<int, int> element : setOfWords){
        ranked_group_frames.push_back(group_frames[element.first]);
    }
}

void cal_subgroup_remove(gm::GlobalMap& map, int N, std::vector<std::vector<std::shared_ptr<gm::Frame>>>& ranked_group_frames){
    std::vector<std::vector<std::shared_ptr<gm::Frame>>> group_frames;
    cal_subgroup(map, group_frames);
    for(int i=0; i<map.frames.size(); i++){
        map.frames[i]->willDel=true;
    }
    for(int n=0; n<N; n++){
        if(group_frames.size()>n){
            ranked_group_frames.push_back(group_frames[n]);
            for(int i=0; i<group_frames[n].size(); i++){
                group_frames[n][i]->willDel=false;
            }
        }
    }
    bool del_one=true;
    int del_count=0;
    while(del_one==true){
        del_one=false;
        for(int i=0; i<map.frames.size(); i++){
            if(map.frames[i]->willDel==true){
                if(map.DelFrame(map.frames[i]->id)){
                    del_count++;
                    del_one=true;
                    break;
                }
            }
        }
    }
    map.AssignKpToMp();
}
