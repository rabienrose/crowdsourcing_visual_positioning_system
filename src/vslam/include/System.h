#ifndef SYSTEM_H
#define SYSTEM_H
#include<opencv2/core/core.hpp>
#include<string>
#include<thread>
#include "Eigen/Dense"
#include "ORBVocabulary.h"
#include "Frame.h"
namespace ORB_SLAM2
{
class Map;
class Tracking;
class LocalMapping;
class LoopClosing;
class KeyFrameDatabase;

class System
{
public:
    System(bool do_loop_detect_flag=true);
    void saveResult(string map_filename);
    bool TrackMonocular(const cv::Mat &im, const double &timestamp, std::string file_name="");
    void getPC(std::vector<Eigen::Vector3d>& pcs);
    void getTraj(std::vector<Eigen::Vector3d>& posis, std::vector<Eigen::Quaterniond>& quas);
    void getDebugImg(cv::Mat& img, float& err, int& count, int & mp_count_, int& kf_count_);
    void saveToVisualMap(string map_filename);
    Frame getCurrentFrame(); 
    Map*  getMapPointer();
    Tracking* getTrackPointer();
private:
    ORBVocabulary* mpVocabulary;
    KeyFrameDatabase* mpKeyFrameDatabase;
    Map* mpMap;
    Tracking* mpTracker;
    LocalMapping* mpLocalMapper;
    LoopClosing* mpLoopCloser;
    int last_kfcount;
    int first_loc_frameid;
};

}// namespace ORB_SLAM

#endif // SYSTEM_H
