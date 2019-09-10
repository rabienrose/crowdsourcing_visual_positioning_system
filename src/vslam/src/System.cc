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
DEFINE_string(map_addr, "", "map file address.");
namespace ORB_SLAM2
{
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
    
    void System::LoadORBMap(std::string mapname, 
                                 ORB_SLAM2::ORBVocabulary*& mpVocabulary, 
                                 ORB_SLAM2::KeyFrameDatabase*& mpKeyFrameDatabase, 
                                 ORB_SLAM2::Map*& mpMap
                   ){
        gm::GlobalMap map;
        std::vector<unsigned int> block_ids;
        block_ids.push_back(112224160);
        gm::load_global_map(map, mapname, block_ids);
        map.AssignKpToMp();
        mpMap = new ORB_SLAM2::Map();
        CHECK_GT(map.frames.size(),0);
        std::vector<float> cam_info;
        cam_info.push_back(map.frames[0]->fx);
        cam_info.push_back(map.frames[0]->fy);
        cam_info.push_back(map.frames[0]->cx);
        cam_info.push_back(map.frames[0]->cy);
        cam_info.push_back(map.frames[0]->width);
        cam_info.push_back(map.frames[0]->height);
        std::map<long unsigned int, ORB_SLAM2::KeyFrame*> kfs;
        for(int i=0; i<map.frames.size(); i++){
            ORB_SLAM2::KeyFrame* pKF = new ORB_SLAM2::KeyFrame();
            cv::Mat pose_c_w=ORB_SLAM2::Converter::toCvMat(map.frames[i]->getPose()).inv();
            int desc_width=map.frames[i]->descriptors.rows();
            int desc_count=map.frames[i]->descriptors.cols();
            cv::Mat desc_mat = cv::Mat(desc_count, desc_width, CV_8UC1);
            for(int j=0; j<desc_width; j++){
                for(int k=0; k<desc_count; k++){
                    desc_mat.at<unsigned char>(k, j) = map.frames[i]->descriptors(j, k);
                } 
            }
            pKF->setData(i, map.frames[i]->time_stamp, map.frames[i]->kps, cam_info, map.frames[i]->frame_file_name, 8, 1.2, pose_c_w, 
                        desc_mat, mpMap, mpKeyFrameDatabase, mpVocabulary);
            //pKF->SetGlobalMapFlag(true);
            mpMap->AddKeyFrame(pKF);
            kfs[map.frames[i]->id]=pKF;
        }
        for(int i=0; i<map.mappoints.size(); i++){
            int mp_id=i;
            ORB_SLAM2::MapPoint* pMP=NULL;
            for (int j=0; j<map.mappoints[i]->track.size(); j++){
                int kp_id=map.mappoints[i]->track[j].kp_ind;
                long unsigned int kpframe_id=map.mappoints[i]->track[j].frame->id;                
                if(pMP==NULL){
                    if(kfs.find(kpframe_id)==kfs.end()){
                        std::cout<<"not exist frame in track"<<std::endl;
                        continue;
                    }
                    pMP = new ORB_SLAM2::MapPoint(ORB_SLAM2::Converter::toCvMat(map.mappoints[i]->position),kfs[kpframe_id],mpMap);
                }

                kfs[kpframe_id]->AddMapPoint(pMP,kp_id);
                pMP->AddObservation(kfs[kpframe_id],kp_id);     
                //pMP->SetGlobalMapFlag(true);
            }
            if(pMP!=NULL){
                mpMap->AddMapPoint(pMP);
            }
        }
        std::vector<ORB_SLAM2::MapPoint*> mps_all=mpMap->GetAllMapPoints();
        for(int i=0; i<mps_all.size(); i++){
            
            mps_all[i]->ComputeDistinctiveDescriptors();
            mps_all[i]->UpdateNormalAndDepth();
        }
        vector<ORB_SLAM2::KeyFrame*> vpKFs = mpMap->GetAllKeyFrames();
        for (vector<ORB_SLAM2::KeyFrame*>::iterator it = vpKFs.begin(); it != vpKFs.end(); ++it){
            (*it)->finishDataSetting();
            mpKeyFrameDatabase->add((*it));
        }
        for (vector<ORB_SLAM2::KeyFrame*>::iterator it = vpKFs.begin(); it != vpKFs.end(); ++it){
            (*it)->UpdateConnections();
        }
        first_loc_frameid=vpKFs.size();
        std::cout<<"map loaded!"<<std::endl;
     }
    
    System::System(bool do_loop_detect_flag, bool loop_for_loc)
    {
        mpVocabulary = new ORBVocabulary();
        LOG(INFO) <<"FLAGS_voc_addr: "<<FLAGS_voc_addr;
        bool bVocLoad= mpVocabulary->loadFromBinaryFile(FLAGS_voc_addr);
        if(bVocLoad==false){
            std::cout<<"try binary voc failed, use txt format to load."<<std::endl;
            mpVocabulary->load(FLAGS_voc_addr);
        }
        mpKeyFrameDatabase = new KeyFrameDatabase(*mpVocabulary);
        if(FLAGS_map_addr!=""){
            LoadORBMap(FLAGS_map_addr, mpVocabulary,  mpKeyFrameDatabase, mpMap);
        }else{
            mpMap = new Map();
        }
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
    
    cv::Mat System::TrackLocalization(const cv::Mat &im, const double &timestamp, std::string file_name)
    {
        cv::Mat Tcw = mpTracker->Loc(im,timestamp, file_name);
        return Tcw;
    }
    
    void System::getPC(std::vector<Eigen::Vector3d>& pcs, bool b_global_mp){
        std::vector<MapPoint*> vpMPs= mpMap->GetAllMapPoints();
        for(int i=0; i<vpMPs.size(); i++){
            MapPoint* pMP=vpMPs[i];
            if(pMP->isBad()){
                continue;
            }
            if(b_global_mp) {
                if(!pMP->GetGlobalMapFlag())
                    continue;
            } else {
                if(pMP->GetGlobalMapFlag())
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
                        cv::circle(img_rgb, obs_pt ,3 ,cv::Scalar(0, 0, 255, 255), 1, 1 ,0);
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
            std::shared_ptr<gm::Frame> frame_p=std::make_shared<gm::Frame>();
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
//
        vector<MapPoint*> vpMPs = mpMap->GetAllMapPoints();
        for(int i=0; i<vpMPs.size(); i++){
            MapPoint* mp = vpMPs[i];
            if (mp->isBad()){
                continue;
            }
            std::shared_ptr<gm::MapPoint> mappoint_p=std::make_shared<gm::MapPoint>();
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

        map.AssignKpToMp();
        gm::save_submap(map, map_filename);
    }
    
    System::~System(){
        mpTracker->Reset();
        delete mpTracker;
        delete mpVocabulary;
        delete mpKeyFrameDatabase;
        delete mpMap;
        delete mpLocalMapper;
        delete mpLoopCloser;
        
    }

} //namespace ORB_SLAM
