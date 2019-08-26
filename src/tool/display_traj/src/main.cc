#include "visualization/color-palette.h"
#include "visualization/color.h"
#include "visualization/common-rviz-visualization.h"
#include <iostream>
#include <fstream>
#include <glog/logging.h>
#include <gflags/gflags.h>
std::vector<std::string> split(const std::string& str, const std::string& delim)
    {
        std::vector<std::string> tokens;
        size_t prev = 0, pos = 0;
        do
        {
            pos = str.find(delim, prev);
            if (pos == std::string::npos) pos = str.length();
            std::string token = str.substr(prev, pos-prev);
            if (!token.empty()) tokens.push_back(token);
            prev = pos + delim.length();
        }
        while (pos < str.length() && prev < str.length());
        return tokens;
    }

void show_mp_as_cloud(std::vector<Eigen::Vector3d>& mp_posis, std::string topic){
    Eigen::Matrix3Xd points;
    points.resize(3,mp_posis.size());
    for(int i=0; i<mp_posis.size(); i++){
        points.block<3,1>(0,i)=mp_posis[i];
    }    
    publish3DPointsAsPointCloud(points, visualization::kCommonRed, 1.0, visualization::kDefaultMapFrame,topic);
}
        
int main(int argc, char* argv[]){
    
    google::InitGoogleLogging(argv[0]);
    google::InstallFailureSignalHandler();
    //google::ParseCommandLineFlags(&argc, &argv, true);
    visualization::RVizVisualizationSink::init();
    CHECK_GT(argc, 5);
    std::string res_root=argv[1];
    int x_index=atoi(argv[2]);
    int y_index=atoi(argv[3]);
    int z_index=atoi(argv[4]);
    int accu_index=atoi(argv[5]);
    std::string show_topic=argv[6];
    std::vector<Eigen::Vector3d> posis;
    std::ifstream infile_pose(res_root.c_str());
    std::string line;
    while (true)
    {
        std::getline(infile_pose, line);
        if (line==""){
            break;
        }
        std::vector<std::string> splited = split(line, ",");
        if(splited.size()<= x_index){
            continue;
        }
        if(splited.size()<= y_index){
            continue;
        }
        if(splited.size()<= z_index){
            continue;
        }
        
        Eigen::Vector3d posi;
        CHECK_GT(splited.size(), x_index);
        CHECK_GT(splited.size(), y_index);
        CHECK_GT(splited.size(), z_index);
        if(accu_index!=-1){
            float accu=atof(splited[accu_index].c_str());
            if(accu<10){
                posi(0)=atof(splited[x_index].c_str());
                posi(1)=atof(splited[y_index].c_str());
                posi(2)=atof(splited[z_index].c_str());
                posis.push_back(posi);
            }
        }else{
            posi(0)=atof(splited[x_index].c_str());
            posi(1)=atof(splited[y_index].c_str());
            posi(2)=atof(splited[z_index].c_str());
            posis.push_back(posi);
        }
    }
    
    show_mp_as_cloud(posis, show_topic);

    ros::spin();

    return 0;
}