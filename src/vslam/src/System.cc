#include "System.h"
#include "Converter.h"
#include <thread>
#include <iomanip>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <Eigen/Dense>
#include "Tracking.h"
#include "Map.h"
#include "LocalMapping.h"
#include "LoopClosing.h"
#include "KeyFrameDatabase.h"
#include <gflags/gflags.h>
#include <glog/logging.h>
#include "global_map/global_map.h"
#include "global_map/global_map_seri.h"
#include "chamo_common/common.h"

DEFINE_string(voc_addr, "", "Vocabulary file address.");

namespace ORB_SLAM2
{
    System::System(bool do_loop_detect_flag)
    {
        mpVocabulary = new ORBVocabulary();
        LOG(INFO) <<"FLAGS_voc_addr: "<<FLAGS_voc_addr;
        bool bVocLoad= mpVocabulary->loadFromBinaryFile(FLAGS_voc_addr);
        if(bVocLoad==false){
            std::cout<<"try binary voc failed, use txt format to load."<<std::endl;
            mpVocabulary->load(FLAGS_voc_addr);
        }
        mpKeyFrameDatabase = new KeyFrameDatabase(*mpVocabulary);
        mpMap = new Map();
        mpTracker = new Tracking(mpVocabulary, mpMap, mpKeyFrameDatabase,0 ,false);
        mpLocalMapper = new LocalMapping(mpMap, true);
        mpLoopCloser = new LoopClosing(mpMap, mpKeyFrameDatabase, mpVocabulary, false);
        mpTracker->SetLocalMapper(mpLocalMapper);
        mpTracker->SetLoopClosing(mpLoopCloser);
        mpLocalMapper->SetTracker(mpTracker);
        mpLocalMapper->SetLoopCloser(mpLoopCloser);
        mpLoopCloser->SetTracker(mpTracker);
        mpLoopCloser->SetLocalMapper(mpLocalMapper);
        mpLocalMapper->SetdoLoop(do_loop_detect_flag);
        last_kfcount=0;
    }
    
    bool System::TrackMonocular(const cv::Mat &im, const double &timestamp, std::string file_name)
    {
        return mpTracker->GrabImageMonocular(im,timestamp, file_name);
    }
    
    Frame System::getCurrentFrame()
    {
        return mpTracker->mCurrentFrame;
    }
    
    Map*  System::getMapPointer()
    {
        return mpMap;
    }

    Tracking* System::getTrackPointer()
    {
        return mpTracker;
    }
    void System::getTraj(std::vector<Eigen::Vector3d>& posis, std::vector<Eigen::Quaterniond>& quas){
        std::vector<KeyFrame*> vpKFs = mpMap->GetAllKeyFrames();
        for(int i=0; i<vpKFs.size(); i++)
        {
            ORB_SLAM2::KeyFrame* pKF = vpKFs[i];
            while(pKF->isBad())
            {
                continue;
            }
            // cv::Mat Two = vpKFs[0]->GetPoseInverse();
            // cv::Mat Trw = pKF->GetPose()*Two;
            cv::Mat Trw = pKF->GetPose();
            cv::Mat Rwc = Trw.rowRange(0,3).colRange(0,3).t();
            cv::Mat twc = -Rwc*Trw.rowRange(0,3).col(3);
            //Eigen::Vector3d posi(twc.at<float>(0),twc.at<float>(2),-twc.at<float>(1));
            Eigen::Vector3d posi(twc.at<float>(0),twc.at<float>(1),twc.at<float>(2));
            posis.push_back(posi);
            Eigen::Quaterniond rot(ORB_SLAM2::Converter::toMatrix3d(Rwc));
            quas.push_back(rot);
        }
    }
    
    void System::getPC(std::vector<Eigen::Vector3d>& pcs){
        std::vector<MapPoint*> vpMPs= mpMap->GetAllMapPoints();
        for(int i=0; i<vpMPs.size(); i++){
            MapPoint* pMP=vpMPs[i];
            if(pMP->isBad()){
                continue;
            }
            Eigen::Vector3d posi = Converter::toVector3d(pMP->GetWorldPos());
            pcs.push_back(posi);
        }
    }
    
    void System::getDebugImg(cv::Mat& img, float& err, int& count, int & mp_count_, int& kf_count_){
        err=0;
        count=0;
        img=cv::Mat();
        std::vector<MapPoint*> vpMPs= mpMap->GetAllMapPoints();
        std::vector<KeyFrame*> vpKFs = mpMap->GetAllKeyFrames();
        mp_count_=vpMPs.size();
        kf_count_=vpKFs.size();
        if(mpTracker->created_new_kf){
            KeyFrame* lastKF = mpTracker->last_kf;
            img=mpTracker->mKfImage;
            cv::Mat pose = lastKF->GetPose();
            Eigen::Matrix4d pose_eig= Converter::toMatrix4d(pose);
            cv::Mat img_rgb;
            cv::cvtColor(img, img_rgb, cv::COLOR_GRAY2BGRA);
            std::vector<MapPoint*> mps= lastKF->GetMapPointMatches();
            double total_error=0;
            for(int i=0; i<mps.size(); i++){
                if(mps[i]!=NULL){
                    if(!mps[i]->isBad()){
                        Eigen::Vector4d posi_mp;
                        cv::Mat posi_mp_cv = mps[i]->GetWorldPos();
                        posi_mp[0]=posi_mp_cv.at<float>(0);
                        posi_mp[1]=posi_mp_cv.at<float>(1);
                        posi_mp[2]=posi_mp_cv.at<float>(2);
                        posi_mp[3]=1;
                        Eigen::Matrix<double, 3, 4> K_eig=Eigen::Matrix<double, 3, 4>::Zero();
                        K_eig(0,0)=lastKF->fx;
                        K_eig(1,1)=lastKF->fy;
                        K_eig(0,2)=lastKF->cx;
                        K_eig(1,2)=lastKF->cy;
                        K_eig(2,2)=1;
                        Eigen::Vector3d projected_pt = K_eig*pose_eig*posi_mp;
                        cv::Point2f obs_pt(projected_pt(0)/projected_pt(2), projected_pt(1)/projected_pt(2));
                        int kp_index = mps[i]->GetIndexInKeyFrame(lastKF);
                        cv::Point2f pt= lastKF->mvKeysUn[kp_index].pt;
                        cv::Point2f d_pt=obs_pt-pt;
                        //std::cout<<pt.x<<","<<pt.y<<" "<<obs_pt.x<<","<<obs_pt.y<<std::endl;
                        total_error=total_error+sqrt(d_pt.x*d_pt.x + d_pt.y*d_pt.y);
                        cv::circle(img_rgb, obs_pt ,3 ,cv::Scalar(0, 255, 255), 1, 1 ,0);
                        count++;
                    }
                }
            }
            err=total_error/count;
            img=img_rgb;
        }
    }
    
    size_t findDesc(std::vector<std::pair<KeyFrame*, size_t>>& target_list, std::pair<KeyFrame*, size_t> query){
        size_t re=-1;
        for(int i=0; i<target_list.size(); i++){
            if(query.first==target_list[i].first && query.second==target_list[i].second){
                if(!target_list[i].first->isBad()){
                    re=i;
                }else{
                    re=0;
                }
                //std::cout<<(int)re<<std::endl;
                break;
            }
        }
        return re;
    }
    
    void System::saveToVisualMap(string map_filename){
        gm::GlobalMap map;
        vector<KeyFrame*> vpKFs = mpMap->GetAllKeyFrames();
        sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);
        for(int i=0; i<vpKFs.size(); i++)
        {
            ORB_SLAM2::KeyFrame* pKF = vpKFs[i];

            if(pKF->isBad())
            {
                continue;
            }

            cv::Mat Trw = pKF->GetPose();
            cv::Mat Rwc = Trw.rowRange(0,3).colRange(0,3).t();
            cv::Mat twc = -Rwc*Trw.rowRange(0,3).col(3);
            Eigen::Vector3d posi(twc.at<float>(0),twc.at<float>(1),twc.at<float>(2));
            Eigen::Quaterniond rot(ORB_SLAM2::Converter::toMatrix3d(Rwc));
            //LOG(INFO)<<pKF->file_name_;
            std::vector<std::string> splited = chamo::split(pKF->file_name_, "/");
            std::string filename= splited.back();
            std::shared_ptr<gm::Frame> frame_p;
            frame_p.reset(new gm::Frame);
            frame_p->time_stamp=pKF->mTimeStamp;
            frame_p->fx=pKF->fx;
            frame_p->fy=pKF->fy;
            frame_p->cx=pKF->cx;
            frame_p->cy=pKF->cy;
            frame_p->k1=0;
            frame_p->k2=0;
            frame_p->p1=0;
            frame_p->p2=0;
            frame_p->frame_file_name=filename;
            frame_p->position=posi;
            frame_p->direction=rot;
            frame_p->kps= pKF->mvKeysUn;
            int desc_width=pKF->mDescriptors.cols;
            int desc_count=pKF->mDescriptors.rows;
            //LOG(INFO)<<pKF->mDescriptors.cols;
            //LOG(INFO)<<pKF->mDescriptors.rows;
            frame_p->descriptors.resize(desc_width, desc_count);
            for(int j=0; j<desc_width; j++){
                for(int k=0; k<desc_count; k++){
                    frame_p->descriptors(j, k)=pKF->mDescriptors.at<unsigned char>(k, j);
                } 
            }
            for(int j=0; j<pKF->mvKeysUn.size(); j++){
                frame_p->obss.push_back(nullptr);
            }
            frame_p->id=pKF->mnId;
            map.frames.push_back(frame_p);
        }
        
        vector<MapPoint*> vpMPs = mpMap->GetAllMapPoints();
        for(int i=0; i<vpMPs.size(); i++){
            MapPoint* mp = vpMPs[i];
            if (mp->isBad()){
                continue;
            }
            std::shared_ptr<gm::MapPoint> mappoint_p;
            mappoint_p.reset(new gm::MapPoint);
            cv::Mat mp_posi_cv=vpMPs[i]->GetWorldPos();
            Eigen::Vector3d posi(mp_posi_cv.at<float>(0),mp_posi_cv.at<float>(1),mp_posi_cv.at<float>(2));
            mappoint_p->position=posi;
            mappoint_p->id=vpMPs[i]->mnId;
            map.mappoints.push_back(mappoint_p);
        }
        for(int i=0; i<vpKFs.size(); i++){
            ORB_SLAM2::KeyFrame* pKF = vpKFs[i];
            if(pKF->isBad()){
                continue;
            }
            if(pKF->mnId!=map.frames[i]->id){
                std::cout<<"[error]pKF->mnId!=frame_p->id"<<std::endl;
                exit(0);
            }
            for(int j=0; j<pKF->mvpMapPoints.size(); j++){
                if(pKF->mvpMapPoints[j]!=NULL){
                    for(int k=0; k<map.mappoints.size(); k++){
                        if(map.mappoints[k]->id==pKF->mvpMapPoints[j]->mnId){
                            map.frames[i]->obss[j]=map.mappoints[k];
                            break;
                        }
                    }
                }
            }
        }
        LOG(INFO)<<"save map: "<<map_filename;
//         for(int i=0; i<vpKFs.size(); i++){
//             ORB_SLAM2::KeyFrame* pKF = vpKFs[i];
//             if(pKF->isBad()){
//                 continue;
//             }
//             std::vector<KeyFrame*> conns = pKF->GetBestCovisibilityKeyFrames(40);
//             std::cout<<pKF->mnId<<std::endl;
//             for(std::vector<KeyFrame*>::iterator it=conns.begin(); it!=conns.end(); it++){
//                 std::cout<<(*it)->mnId<<":";
//             }
//             std::cout<<std::endl;
//         }
        
//         for(int i=0; i<vpKFs.size(); i++){
//             ORB_SLAM2::KeyFrame* pKF = vpKFs[i];
//             if(pKF->isBad()){
//                 continue;
//             }
//             std::map<KeyFrame*, int> frame_list;
//             for(int j=0; j<vpKFs[i]->mvpMapPoints.size(); j++){
//                 if(pKF->mvpMapPoints[j]!=NULL){
//                     std::map<KeyFrame*,size_t> observations = pKF->mvpMapPoints[j]->GetObservations();
// 
//                     for(std::map<KeyFrame*,size_t>::iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
//                     {
//                         if(mit->first->mnId==pKF->mnId)
//                             continue;
//                         frame_list[mit->first]++;
//                     }
//                 }
//             }
//             std::cout<<"=========="<<pKF->mnId<<"=========="<<std::endl;
//             for(std::map<KeyFrame*,int>::iterator mit=frame_list.begin(), mend=frame_list.end(); mit!=mend; mit++)
//             {
//                 std::cout<<mit->first->mnId<<" : "<<mit->second<<std::endl;
//             }
//         }
        map.AssignKpToMp();
//         for(int i=0; i<map.frames.size(); i++){
//             std::map<std::shared_ptr<gm::Frame>, int> frame_list;
//             for(int j=0; j<map.frames[i]->obss.size(); j++){
//                 
//                 if(map.frames[i]->obss[j]!=nullptr){
//                     
//                     for(int k=0; k<map.frames[i]->obss[j]->track.size(); k++){
//                         frame_list[map.frames[i]->obss[j]->track[k].frame]++;
//                     }
//                 }
//             }
//             std::cout<<"=========="<<map.frames[i]->id<<"=========="<<std::endl;
//             for(std::map<std::shared_ptr<gm::Frame>,int>::iterator mit=frame_list.begin(), mend=frame_list.end(); mit!=mend; mit++)
//             {
//                 std::cout<<mit->first->id<<" : "<<mit->second<<std::endl;
//             }
//         }
        map.CalConnections();
        gm::save_submap(map, map_filename);
    }

} //namespace ORB_SLAM
