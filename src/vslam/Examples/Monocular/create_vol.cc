#include <iostream>
#include <stdio.h>
#include <vector>

#include "TemplatedVocabulary.h"
#include "BowVector.h"
#include "FeatureVector.h"
#include "FORB.h"
#include "FFREAK.h"

#include "ORBmatcher.h"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <dirent.h>

typedef DBoW2::TemplatedVocabulary<DBoW2::FFREAK::TDescriptor, DBoW2::FFREAK> FreakVocabulary;
typedef DBoW2::TemplatedVocabulary<DBoW2::FORB::TDescriptor, DBoW2::FORB> OrbVocabulary;
  
int getdir (std::string dir, std::vector<std::string> &files)
{
    DIR *dp;
    struct dirent *dirp;
    if((dp  = opendir(dir.c_str())) == NULL) {
        std::cout << "Error opening " << dir << std::endl;
        return 0;
    }

    while ((dirp = readdir(dp)) != NULL) {
        files.push_back(std::string(dirp->d_name));
    }
    closedir(dp);
    return 0;
}
  
void changeStructure(const cv::Mat &plain, vector<cv::Mat> &out)
{
    out.resize(plain.rows);

    for(int i = 0; i < plain.rows; ++i)
    {
        out[i] = plain.row(i);
    }
}

int main(int argc,char **argv)
{
    std::string out_file=argv[1];
    std::string desc_type=argv[2];
    int max_img=atoi(argv[3]);
    int k=atoi(argv[4]);
    int L=atoi(argv[5]);
    std::vector<std::string> img_root;
    for (int i=6; i<argc; i++){
        img_root.push_back(argv[i]);
        std::cout<<argv[i]<<std::endl;
    }
    
    int total_file_count=0;
    for(int j=0; j<img_root.size(); j++){
        std::vector<std::string> files;
        getdir(img_root[j], files);
        total_file_count=total_file_count+files.size();
    }
    int step=total_file_count/max_img;
    if(step<0){
        std::cout<<"[create voc][error]step<0"<<std::endl;
        return 0;
    }
    std::vector<std::string> file_list_vec;
    for(int j=0; j<img_root.size(); j++){
        std::vector<std::string> files;
        getdir(img_root[j], files);
        for(int k=0; k<files.size(); k=k+step){
            file_list_vec.push_back(img_root[j]+"/"+files[k]);
        }
    }
    
    std::cout<<"get all image names: "<<file_list_vec.size()<<std::endl;
    
    std::vector<std::vector<cv::Mat>> features;
    ORB_SLAM2::ORBextractor mpORBextractor(2000, 1.2, 8, 20, 7);
    for(size_t i = 0; i < file_list_vec.size(); ++i)
    {
        cv::Mat descriptors;
        std::vector<cv::KeyPoint> keypoints;
        cv::Mat image = cv::imread(file_list_vec[i], 0);
        if(image.empty()){
            continue;
        }
        bool is_orb=true;
        if(desc_type!="orb"){
            is_orb=false;
        }
        mpORBextractor.ExtractDesc(image, cv::Mat() ,keypoints, descriptors, is_orb);
        std::vector<cv::Mat> out;
        changeStructure(descriptors, out);
        features.push_back(out);
    }
    
    std::cout<<"extract desc finished"<<std::endl;
    
    const DBoW2::WeightingType weight = DBoW2::TF_IDF;
    const DBoW2::ScoringType score = DBoW2::L1_NORM;
    if(desc_type=="orb"){
        OrbVocabulary voc(k, L, weight, score);
        voc.create(features);
        voc.saveToBinaryFile(out_file);
    }else{
        FreakVocabulary voc(k, L, weight, score);
        voc.create(features);
        voc.saveToBinaryFile(out_file);
    }

    return 0;
}