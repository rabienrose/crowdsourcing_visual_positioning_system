#include <string>
#include <fstream>
#include <memory>
#include <opencv2/opencv.hpp>
#include <Eigen/Core>
#include "global_map/global_map.h"
#include <math.h>
#include "global_map/global_map_seri.h"
#include "chamo_common/common.h"
#include <glog/logging.h>
#include <gflags/gflags.h>
#include <backend/header.h>
#include <sim3_ransac/sim3_match.h>
#include "imu_tools.h"
#include "NavState.h"
#include "IMUConverter.h"
#include "CoorConv.h"
#include "convert_to_visual_map/convert_to_visual_map.h"

DEFINE_double(err_thres, 0.1, "if the sim3 match err is large than this, the trajectory will be cut");

void updatePreInt(std::vector<chamo::IMUPreintegrator>& preints, std::vector<std::vector<chamo::IMUData>>& sycn_imu_datas,
        Eigen::Vector3d ba, Eigen::Vector3d bg
    ){
    for(int i=0; i<sycn_imu_datas.size(); i++){
        chamo::IMUPreintegrator temp_preint;
        temp_preint.reset();
        for(int j=1; j<sycn_imu_datas[i].size(); j++){
            double dt=sycn_imu_datas[i][j]._t-sycn_imu_datas[i][j-1]._t;
            temp_preint.update(sycn_imu_datas[i][j]._g - bg, sycn_imu_datas[i][j]._a - ba, dt);
        }
        preints.push_back(temp_preint);
    }
}

void alignToIMU(gm::GlobalMap& map){
    CHECK_GT(map.frames.size(), 0);
    Eigen::Matrix3d temp_rot(map.frames[0]->Tbc_qua);
    Eigen::Matrix4d Tbc= Eigen::Matrix4d::Identity();
    Tbc.block(0,0,3,3)=temp_rot;
    Tbc.block(0,3,3,1)=map.frames[0]->Tbc_posi;
    std::vector<std::vector<chamo::IMUData>> sycn_imu_datas;
    std::vector<cv::Mat> pose_vec_mat;
    for(int i=0; i<map.frames.size(); i++){
        pose_vec_mat.push_back(chamo::Converter::toCvMat(map.frames[i]->getPose()));
        std::vector<chamo::IMUData> imudatas;
        sycn_imu_datas.push_back(imudatas);
    }
    for(int i=1; i<map.frames.size(); i++){
        CHECK_EQ(map.frames[i-1]->acces.size(), map.frames[i-1]->gyros.size());
        CHECK_EQ(map.frames[i-1]->acces.size(), map.frames[i-1]->imu_times.size());
        
        std::vector<chamo::IMUData> imudatas;
        if(map.frames[i]->acces.size()>0){
            for(int j=0; j<map.frames[i-1]->acces.size(); j++){
                chamo::IMUData imu_data;
                imu_data._a=map.frames[i-1]->acces[j];
                imu_data._g=map.frames[i-1]->gyros[j];
                imu_data._t=map.frames[i-1]->imu_times[j];
                sycn_imu_datas[i].push_back(imu_data);
            }       
            chamo::IMUData imu_data;
            imu_data._a=map.frames[i]->acces[0];
            imu_data._g=map.frames[i]->gyros[0];
            imu_data._t=map.frames[i]->imu_times[0];
            sycn_imu_datas[i].push_back(imu_data);
        }
    }
    Eigen::Vector3d bg=Eigen::Vector3d::Zero();
    Eigen::Vector3d ba=Eigen::Vector3d::Zero();
    std::vector<chamo::IMUPreintegrator> preints;
    updatePreInt(preints, sycn_imu_datas, ba, bg);
    Eigen::Vector3d new_bg = OptimizeInitialGyroBias(pose_vec_mat, preints, Tbc);
    bg=new_bg;
    preints.clear();
    updatePreInt(preints, sycn_imu_datas, ba, bg);
    double sstar;
    cv::Mat gwstar;
    double scale_confi;
    double grav_confi;
    CalGravityAndScale(pose_vec_mat, preints, chamo::Converter::toCvMat(Tbc), sstar, gwstar, scale_confi, grav_confi);
    cv::Mat gI = cv::Mat::zeros(3,1,CV_32F);
    gI.at<float>(2) = 1;
    cv::Mat gwn = gwstar/cv::norm(gwstar);
    cv::Mat gIxgwn = gI.cross(gwn);
    double normgIxgwn = cv::norm(gIxgwn);
    cv::Mat vhat = gIxgwn/normgIxgwn;
    double theta = std::atan2(normgIxgwn,gI.dot(gwn));
    Eigen::Vector3d vhateig = chamo::Converter::toVector3d(vhat);
    Eigen::Matrix3d Rwi_ = Sophus::SO3::exp(vhateig*theta).matrix();
    Eigen::Vector3d bias_a=Eigen::Vector3d::Zero();
    CalAccBias(pose_vec_mat, preints, sstar, gwstar, chamo::Converter::toCvMat(Tbc), Rwi_, bias_a);
    cv::Mat Tbc_mat=chamo::Converter::toCvMat(Tbc);
    cv::Mat Rwi_mat=chamo::Converter::toCvMat(Rwi_);
    Eigen::Matrix4d Twi = Eigen::Matrix4d::Identity();
    Twi.block(0,0,3,3)=Rwi_;
    cv::Mat Twi_mat=chamo::Converter::toCvMat(Twi);
    Eigen::Vector3d last_v;
    for(int i=0; i<pose_vec_mat.size(); i++){
        pose_vec_mat[i].col(3).rowRange(0,3)=pose_vec_mat[i].col(3).rowRange(0,3)*sstar;
        pose_vec_mat[i]=Twi_mat.t()*pose_vec_mat[i];
        cv::Mat wPc = pose_vec_mat[i].rowRange(0,3).col(3);                   // wPc
        cv::Mat Rwc = pose_vec_mat[i].rowRange(0,3).colRange(0,3);            // Rwc
        map.frames[i]->position=chamo::Converter::toVector3d(wPc);
        Eigen::Matrix3d tempm=chamo::Converter::toMatrix3d(Rwc);
        map.frames[i]->direction=Eigen::Quaterniond(tempm);
    }
    for(int i=0; i<map.mappoints.size(); i++){
        map.mappoints[i]->position=Rwi_.transpose()*map.mappoints[i]->position*sstar;
    }
    
}

void transformPoseUseSim3(Eigen::Matrix4d& sim3, double scale,  Eigen::Matrix4d& in_pose,  Eigen::Matrix4d& out_pose){
    Eigen::Matrix3d R_tran=sim3.block(0,0,3,3)/scale;
    Eigen::Matrix3d R_in=in_pose.block(0,0,3,3);
    Eigen::Matrix3d R_out=R_tran*R_in;
    Eigen::Vector4d t_out=sim3*in_pose.block(0,3,4,1);
    out_pose= Eigen::Matrix4d::Identity();
    out_pose.block(0,0,3,3) = R_out;
    out_pose.block(0,3,4,1) = t_out;
}

void alignToGPS(gm::GlobalMap& map, std::vector<gm::GlobalMap>& out_maps){
    int last_frame_id=0;
    for(int nn=10; nn<map.frames.size(); nn++){
        int cur_frame_id=nn;
        std::vector<Eigen::Vector3d> pc_frame;
        std::vector<Eigen::Vector3d> pc_gps;
        for(int i=last_frame_id+1; i<cur_frame_id; i++){
            std::shared_ptr<gm::Frame> frame = map.frames[i];
            if(frame->gps_accu<30){
                pc_frame.push_back(frame->position);
                pc_gps.push_back(frame->gps_position);
            }else{
                //std::cout<<"cannot find frame name in pose list: "<<std::endl;
            }
        }

        if(pc_gps.size()>=10){
            double scale_12;
            Eigen::Matrix4d T12;
            chamo::ComputeSim3(pc_gps, pc_frame , T12, scale_12);
            
            float avg_err=0;
            std::vector<Eigen::Vector3d> pc_frame_transformed_temp;
            for(int i=0; i<pc_gps.size(); i++){
                Eigen::Vector4d posi_homo;
                posi_homo.block(0,0,3,1)=pc_frame[i];
                posi_homo(3)=1;
                Eigen::Vector4d posi_gps_homo = T12*posi_homo;
                float err = (pc_gps[i]-posi_gps_homo.block(0,0,3,1)).norm();
                avg_err=avg_err+err/pc_gps.size();
                pc_frame_transformed_temp.push_back(posi_gps_homo.block(0,0,3,1));
            }
            //std::cout<<"avg_err: "<<cur_frame_id<<" | "<<pc_gps.size()<<" | "<<avg_err<<std::endl;
            //if(avg_err>FLAGS_err_thres || cur_frame_id==map.frames.size()-1){
            if(cur_frame_id==map.frames.size()-1){
                gm::GlobalMap submap;
                map.CreateSubMap(last_frame_id, cur_frame_id, submap);
                for(int j=0; j<submap.frames.size(); j++){
                    Eigen::Matrix4d pose_transformed_temp;
                    Eigen::Matrix4d temp_pose=submap.frames[j]->getPose();
                    transformPoseUseSim3(T12, scale_12, temp_pose, pose_transformed_temp);
                    submap.frames[j]->setPose(pose_transformed_temp);
                }
                for(int j=0; j<submap.mappoints.size(); j++){
                    Eigen::Vector4d posi_homo;
                    posi_homo.block(0,0,3,1)=submap.mappoints[j]->position;
                    posi_homo(3)=1;
                    Eigen::Vector4d posi_gps_homo = T12*posi_homo;
                    submap.mappoints[j]->position=posi_gps_homo.block(0,0,3,1);                       
                }
                out_maps.push_back(submap);
                last_frame_id=cur_frame_id;
            }
        }
    }
    if(out_maps.size()==0){
        out_maps.push_back(map);
    }
}

void read_imu_data(std::string imu_addr, std::vector<Eigen::Matrix<double, 7, 1>>& imu_datas){
    std::string line;
    std::ifstream infile_imu(imu_addr.c_str());
    int imu_count=0;
    Eigen::Vector3d gravity(0,0,-9.8);
    Eigen::Vector3d bg=Eigen::Vector3d::Zero();
    Eigen::Vector3d ba=Eigen::Vector3d::Zero();
    
    while (true)
    {
        std::getline(infile_imu, line);
        if (line==""){
            break;
        }
        std::vector<std::string> splited = chamo::split(line, ",");
        Eigen::Matrix<double, 7, 1> imu;
        for(int i=0; i<7; i++){
            imu(i)=atof(splited[i].c_str());
        }
        imu_datas.push_back(imu);
    }
}

void read_cam_info(std::string cam_addr, Eigen::Matrix3d& cam_inter, Eigen::Vector4d& cam_distort, Eigen::Matrix4d& Tbc){
    std::string line;
    std::ifstream infile_camera(cam_addr.c_str()); 
    std::getline(infile_camera, line);
    std::vector<std::string> splited = chamo::split(line, ",");
    cam_inter=Eigen::Matrix3d::Identity();
    cam_inter(0,0)=atof(splited[0].c_str());
    cam_inter(1,1)=atof(splited[1].c_str());
    cam_inter(0,2)=atof(splited[2].c_str());
    cam_inter(1,2)=atof(splited[3].c_str());
    cam_distort(0)=atof(splited[4].c_str());
    cam_distort(1)=atof(splited[5].c_str());
    cam_distort(2)=atof(splited[6].c_str());
    cam_distort(3)=atof(splited[7].c_str());
    Tbc=Eigen::Matrix4d::Identity();
    std::getline(infile_camera, line);
    if(line.size()<=0){
        LOG(INFO) << "Not use camera to imu transformation!";
        return;
    }
    splited = chamo::split(line, ",");
    Tbc(0,0)=atof(splited[0].c_str());
    Tbc(0,1)=atof(splited[1].c_str());
    Tbc(0,2)=atof(splited[2].c_str());
    Tbc(0,3)=atof(splited[3].c_str());
    Tbc(1,0)=atof(splited[4].c_str());
    Tbc(1,1)=atof(splited[5].c_str());
    Tbc(1,2)=atof(splited[6].c_str());
    Tbc(1,3)=atof(splited[7].c_str());
    Tbc(2,0)=atof(splited[8].c_str());
    Tbc(2,1)=atof(splited[9].c_str());
    Tbc(2,2)=atof(splited[10].c_str());
    Tbc(2,3)=atof(splited[11].c_str());
}

void read_img_time(std::string img_time_addr, std::vector<double>& img_timess, std::vector<std::string>& img_names){
    std::string line;
    std::ifstream infile(img_time_addr);
    while (true)
    {
        std::getline(infile, line);
        if (line==""){
            break;
        }
        std::vector<std::string> splited = chamo::split(line, ",");
        std::string img_name=splited[0];
        double img_time=atof(splited[1].c_str());
        img_timess.push_back(img_time);
        img_names.push_back(img_name);
    }
    infile.close();
}

void read_gps_alin(std::string gps_alin_addr, std::vector<Eigen::Vector3d>& gps_alins, std::vector<int>& inliars, std::vector<float>& accus){
    std::string line;
    std::ifstream infile(gps_alin_addr);
    while (true)
    {
        std::getline(infile, line);
        if (line==""){
            break;
        }
        std::vector<std::string> splited = chamo::split(line, ",");
        if(splited.size()==2){
            Eigen::Vector3d temp=Eigen::Vector3d::Zero();
            gps_alins.push_back(temp);
            accus.push_back(9999);
            inliars.push_back(0);
        }else{
            Eigen::Vector3d temp;
            temp(0)=atof(splited[2].c_str());
            temp(1)=atof(splited[3].c_str());
            temp(2)=atof(splited[4].c_str());
            accus.push_back(atof(splited[5].c_str()));
            gps_alins.push_back(temp);
            inliars.push_back(1);
        }
    }
    infile.close();
}
     

void findFramePoseByName(std::vector<std::string>& names, int& re_id, std::string query_name){
    re_id=-1;
    for(int i=0; i<names.size(); i++){
        if(names[i]==query_name){
            re_id=i;
            return;
        }
    }
    return;
}

void findAllKP(std::string frame_name, std::vector<std::string>& kp_framename, std::vector<int>& old_kp_id_out){
    for(int i=0; i<kp_framename.size(); i++){
        if(kp_framename[i]==frame_name){
            old_kp_id_out.push_back(i);
        }
    }
}

Eigen::Vector3d interEigenV(Eigen::Vector3d v1, Eigen::Vector3d v2, double t1, double t2, double t3){
    return v1 + (v2 - v1) * (t3 - t1) / (t2 - t1);
}

class IMUData_t
{
public:
    IMUData_t(const double& gx, const double& gy, const double& gz,
                 const double& ax, const double& ay, const double& az,
                 const double& t) :_g(gx,gy,gz), _a(ax,ay,az), _t(t){}
    IMUData_t(){};
    Eigen::Vector3d _g;    //gyr data
    Eigen::Vector3d _a;    //acc data
    double _t;      //timestamp
};

long unsigned int calIdFromGPS(double lat, double lon){
    return floor(lat*100)*floor(lon*100);
}

void ConvertFromVisualMap(std::string config_root, std::string res_root, gm::GlobalMap& map, std::vector<gm::GlobalMap>& out_maps){
    
    std::string img_time_addr=res_root+"/image_time.txt";
    std::vector<double> img_timess;
    std::vector<std::string> img_names;
    read_img_time(img_time_addr, img_timess, img_names);
    std::cout<<"img_timess: "<<img_timess.size()<<std::endl;
    
    std::string gps_alin_addr=res_root+"/gps_alin.txt";
    std::vector<int> gps_inliers;
    std::vector<float> gps_accus;
    std::vector<Eigen::Vector3d> gps_alins;
    read_gps_alin(gps_alin_addr, gps_alins, gps_inliers, gps_accus);
    std::vector<Eigen::Vector3d> gps_alins_xyz;
    bool get_anchor=false;
    for(int i=0; i<gps_alins.size(); i++){
        Eigen::Vector3d coor_gps;
        if(get_anchor==false){
            if(gps_accus[i]<100){
                map.gps_anchor=gps_alins[i];
                get_anchor=true;
                convert_to_coor(gps_alins[i], coor_gps, map.gps_anchor);
            }
        }else{
            convert_to_coor(gps_alins[i], coor_gps, map.gps_anchor);
        }
        gps_alins_xyz.push_back(coor_gps);
    }
    for(int i=0; i<gps_alins_xyz.size(); i++){
        if(gps_accus[i]>=100){
            gps_alins_xyz[i]=map.gps_anchor;
        }
    }
    std::cout<<"gps_alins: "<<gps_alins.size()<<std::endl;
    if(gps_alins.size()!=img_names.size()){
        std::cout<<"gps count not equal frame count!!!"<<std::endl;
        exit(0);
    }

    Eigen::Matrix3d cam_inter;
    Eigen::Vector4d cam_distort;
    Eigen::Matrix4d Tbc;
    read_cam_info(config_root+"/camera_config.txt", cam_inter, cam_distort, Tbc);
    
    std::string imu_addr=res_root+"/imu.txt";
    std::vector<Eigen::Matrix<double, 7, 1>> imu_datas_raw;
    read_imu_data(imu_addr, imu_datas_raw);
    
    std::vector<IMUData_t> imu_datas;
    for(int i=0; i<imu_datas_raw.size(); i++){
        double timestamp=imu_datas_raw[i](0);
        double gx=imu_datas_raw[i](1);
        double gy=imu_datas_raw[i](2);
        double gz=imu_datas_raw[i](3);
        double ax=imu_datas_raw[i](4);
        double ay=imu_datas_raw[i](5);
        double az=imu_datas_raw[i](6);
        IMUData_t imu_data(gx, gy, gz, ax, ay, az, timestamp);
        imu_datas.push_back(imu_data);
    }
    
    std::vector<std::vector<IMUData_t>> sycn_imu_datas_all;
    int procceing_imu_id=1;
    for(int j=0; j<map.frames.size(); j++){
        double time = map.frames[j]->time_stamp;
        map.frames[j]->Tbc_posi=Tbc.block(0,3,3,1);
        Eigen::Matrix3d rot_t=Tbc.block(0,0,3,3);
        map.frames[j]->Tbc_qua=Eigen::Quaterniond(rot_t);
        std::vector<IMUData_t> temp_imu_dat;
        bool finish_all_imu=false;
        while(true){
            int i=procceing_imu_id;
            if(imu_datas[i]._t>=time && imu_datas[i-1]._t<time){
                IMUData_t imu_data_temp;
                imu_data_temp._g = interEigenV(imu_datas[i-1]._g, imu_datas[i]._g, imu_datas[i-1]._t, imu_datas[i]._t, time);
                imu_data_temp._a = interEigenV(imu_datas[i-1]._a, imu_datas[i]._a, imu_datas[i-1]._t, imu_datas[i]._t, time);
                imu_data_temp._t = time;
                temp_imu_dat.push_back(imu_data_temp);
                break;
            }else{
                if(imu_datas[i]._t<=time){
                    temp_imu_dat.push_back(imu_datas[i]);
                    procceing_imu_id++;
                    if(procceing_imu_id>=imu_datas.size()){
                        finish_all_imu=true;
                    }
                }else{
                    break;
                }
            }
        }
        sycn_imu_datas_all.push_back(temp_imu_dat);
        if(finish_all_imu==true){
            break;
        }
    }
    CHECK_EQ(sycn_imu_datas_all.size(), map.frames.size());
    for(int i=0; i<sycn_imu_datas_all.size()-1; i++){
        if(sycn_imu_datas_all[i+1].size()>0){
            if(sycn_imu_datas_all[i].size()==0){
                continue;
            }
            map.frames[i]->imu_next_frame=map.frames[i+1];
            map.frames[i]->acces.push_back(sycn_imu_datas_all[i].back()._a);
            map.frames[i]->gyros.push_back(sycn_imu_datas_all[i].back()._g);
            map.frames[i]->imu_times.push_back(sycn_imu_datas_all[i].back()._t);
        }
        for(int j=0; j<sycn_imu_datas_all[i+1].size()-1; j++){
            map.frames[i]->acces.push_back(sycn_imu_datas_all[i+1][j]._a);
            map.frames[i]->gyros.push_back(sycn_imu_datas_all[i+1][j]._g);
            map.frames[i]->imu_times.push_back(sycn_imu_datas_all[i+1][j]._t);
        }
    }
    //include the first frame imudata between two frames, not include the last frame imudata.
    //not include the imudat after the last frame
    
    for(int i=0;i<map.frames.size(); i++){
        int time_id=-1;
        findFramePoseByName(img_names, time_id, map.frames[i]->frame_file_name);
        CHECK_NE(time_id, -1);
        CHECK_GT(gps_alins_xyz.size(), time_id);
        map.frames[i]->time_stamp=img_timess[time_id];
        map.frames[i]->gps_position=gps_alins_xyz[time_id];
        map.frames[i]->gps_accu=gps_accus[time_id];
    }

    alignToIMU(map);
    alignToGPS(map, out_maps);
}

void convert_to_visual_map(std::string config_root, std::string res_root, std::string globalmap_root, std::vector<unsigned int>& ids) {
    gm::GlobalMap global_map;
    srand(time(0));
    for(int i=0; i<1000; i++){
        std::stringstream ss;
        ss<<i;
        std::string map_name="submap_"+ss.str()+".map";
        gm::GlobalMap map;
        bool re= gm::load_submap(map, res_root+"/"+map_name, true);
        map.AssignKpToMp();
        if(re==false){
            break;
        }
        if(map.frames.size()<3){
            continue;
        }
        std::vector<gm::GlobalMap> out_maps;
        ConvertFromVisualMap(config_root, res_root, map, out_maps);
        for(int n=0; n<out_maps.size(); n++){
            for(int i=0; i<out_maps[n].frames.size(); i++){     
                long unsigned int new_id;
                Eigen::Vector3d gps_latlon;
                convert_to_lonlat(out_maps[n].frames[i]->position, gps_latlon, map.gps_anchor);
                gm::get_new_global_id(new_id, gps_latlon);
                out_maps[n].frames[i]->id=new_id;
                global_map.gps_anchor=out_maps[n].gps_anchor;
                global_map.frames.push_back(out_maps[n].frames[i]);
            }
            for(int i=0; i<out_maps[n].mappoints.size(); i++){
                long unsigned int new_id;
                Eigen::Vector3d gps_latlon;
                convert_to_lonlat(out_maps[n].mappoints[i]->position, gps_latlon, map.gps_anchor);
                gm::get_new_global_id(new_id, gps_latlon);
                out_maps[n].mappoints[i]->id=new_id;
                global_map.mappoints.push_back(out_maps[n].mappoints[i]);
            }
        }
    }
    gm::get_blockid_list(global_map, ids);
    if(global_map.frames.size()>3){
        gm::save_global_map(global_map, globalmap_root);
    }
}
