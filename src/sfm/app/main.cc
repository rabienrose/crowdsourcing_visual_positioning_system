#include <cstdio>
#include <iostream>
#include <string>
#include <sstream>
#include <iostream>
#include <fstream>
#include <map>
#include "controllers/automatic_reconstruction.h"
#include "base/reconstruction.h"
#include <gflags/gflags.h>
#include "ros/ros.h"
#include "visualization/color-palette.h"
#include "visualization/color.h"
#include "visualization/common-rviz-visualization.h"
#include "util/ply.h"
#include "opencv2/highgui/highgui.hpp"
#include "base/image_reader.h"
#include "base/gps.h"
#include "base/pose.h"
#include "CoorConv.h"
#include "sim3_ransac/sim3_match.h"
#include "chamo_common/common.h"
void show_mp_as_cloud(std::vector<Eigen::Vector3d>& mp_posis, std::string topic){
    Eigen::Matrix3Xd points;
    points.resize(3,mp_posis.size());
    for(int i=0; i<mp_posis.size(); i++){
        points.block<3,1>(0,i)=mp_posis[i];
    }    
    publish3DPointsAsPointCloud(points, visualization::kCommonWhite, 1.0, visualization::kDefaultMapFrame,topic);
}


DEFINE_string(image_path, "", "");
DEFINE_string(vocab_tree_path, "", "");
DEFINE_string(workspace_path, "", "");
DEFINE_string(op_type, "", "");

int main(int argc, char **argv){
    google::InitGoogleLogging(argv[0]);
    google::InstallFailureSignalHandler();
    google::ParseCommandLineFlags(&argc, &argv, true);
    std::cout<<FLAGS_op_type<<std::endl;
    if(FLAGS_op_type=="map"){
        colmap::AutomaticReconstructionController::Options options;
        options.workspace_path=FLAGS_workspace_path;
        options.image_path=FLAGS_image_path;
        options.vocab_tree_path=FLAGS_vocab_tree_path;
        options.data_type = colmap::AutomaticReconstructionController::DataType::INDIVIDUAL;
        options.quality = colmap::AutomaticReconstructionController::Quality::HIGH;
        options.single_camera = true;
        //options.camera_model = "OPENCV";
        //options.camera_params = "1000, 1000, 645, 464, 0.0256, -0.0284, -0.0025, 0.0022";
        options.sparse = true;
        options.dense = true;
        options.mesher = colmap::AutomaticReconstructionController::Mesher::POISSON;
        options.num_threads = -1;
        options.use_gpu = true;
        options.gpu_index = "-1";
        options.snapshot_path = "/home/chamo/Documents/data/ws_peng/snapmap";
        colmap::ReconstructionManager reconstruction_manager;
        colmap::AutomaticReconstructionController app(options, &reconstruction_manager);
        app.Run();
    }else if(FLAGS_op_type=="gps"){
        visualization::RVizVisualizationSink::init();
        colmap::Reconstruction reconstruction;
        reconstruction.Read(FLAGS_workspace_path);
        std::vector<Eigen::Vector3d> gps_list;
        std::vector<Eigen::Vector3d> posi_list;
        Eigen::Vector3d anchor_gps(-1, -1 ,-1);
        for(auto id: reconstruction.RegImageIds()){
            colmap::Image& image_t= reconstruction.Image(id);
            //std::cout<<image_t.Name()<<std::endl;
            std::string full_image_addr = FLAGS_image_path+"/"+image_t.Name();
            colmap::ImageReader reader;
            Eigen::Vector3d gps = reader.GetGPS(full_image_addr);
            Eigen::Vector3d coor_gps;
            if(anchor_gps(0)==-1){
                anchor_gps=gps;
            }
            convert_to_coor(gps, coor_gps, anchor_gps);
            gps_list.push_back(coor_gps);
            posi_list.push_back(image_t.ProjectionCenter());
            //std::cout<<std::setprecision(15)<<coor_gps.transpose()<<std::endl;
        }
        Eigen::Matrix4d T12i_eig;
        double scale;
        chamo::ComputeSim3(gps_list, posi_list, T12i_eig, scale);
        float avg_err=0;
        std::vector<Eigen::Vector3d> pc_frame_transformed_temp;
        std::vector<Eigen::Vector3d> trans_posis;
        std::vector<Eigen::Vector3d> trans_mps;
        for(int i=0; i<reconstruction.RegImageIds().size(); i++){
            colmap::Image& image_t= reconstruction.Image(reconstruction.RegImageIds()[i]);
            Eigen::Matrix4d pose_transformed_temp;
            Eigen::Matrix4d temp_pose=Eigen::Matrix4d::Identity();
            temp_pose.block(0,0,3,3)=image_t.RotationMatrix();
            temp_pose.block(0,3,3,1)=image_t.ProjectionCenter();
            chamo::transformPoseUseSim3(T12i_eig, temp_pose, pose_transformed_temp);
            image_t.SetTvec(pose_transformed_temp.block(0,3,3,1));
            image_t.SetQvec(colmap::RotationMatrixToQuaternion(pose_transformed_temp.block(0,0,3,3)));
            trans_posis.push_back(pose_transformed_temp.block(0,3,3,1));
        }
        for (auto id: reconstruction.Point3DIds()){
            colmap::Point3D pt= reconstruction.Point3D(id);
            Eigen::Vector3d temp_3d(pt.X(), pt.Z(), -pt.Y());
            Eigen::Vector4d posi_homo;
            posi_homo.block(0,0,3,1)=temp_3d;
            posi_homo(3)=1;
            Eigen::Vector4d posi_gps_homo = T12i_eig*posi_homo;
            reconstruction.Point3D(id).SetXYZ(posi_gps_homo.block(0,0,3,1));
            trans_mps.push_back(posi_gps_homo.block(0,0,3,1));
        }
        show_mp_as_cloud(gps_list, "gps_cloud");
        show_mp_as_cloud(trans_posis, "trans_posi");
        show_mp_as_cloud(trans_mps, "trans_mp");
        ros::spin();
    }else if(FLAGS_op_type=="loc"){
        visualization::RVizVisualizationSink::init();
        colmap::Reconstruction reconstruction;
        reconstruction.Read(FLAGS_workspace_path);
        
    }else if(FLAGS_op_type=="show"){
        visualization::RVizVisualizationSink::init();
        colmap::Reconstruction reconstruction;
        reconstruction.Read(FLAGS_workspace_path);
        std::cout<<reconstruction.NumPoints3D()<<std::endl;
        std::vector<Eigen::Vector3d> mp_posis;
        std::vector<Eigen::Vector3d> cam_verts;
        double cam_scale=0.3;
        double cam_w_2=1*cam_scale;
        double cam_h_2=0.5*cam_scale;
        double cam_d =1.5*cam_scale;
        cam_verts.push_back(Eigen::Vector3d(0, 0, 0));
        cam_verts.push_back(Eigen::Vector3d(cam_w_2, cam_h_2, cam_d));
        cam_verts.push_back(Eigen::Vector3d(-cam_w_2, cam_h_2, cam_d));
        cam_verts.push_back(Eigen::Vector3d(-cam_w_2, -cam_h_2, cam_d));
        cam_verts.push_back(Eigen::Vector3d(cam_w_2, -cam_h_2, cam_d));
        for (auto id: reconstruction.Point3DIds()){
            colmap::Point3D pt= reconstruction.Point3D(id);
            Eigen::Vector3d temp_3d(pt.X(), pt.Z(), -pt.Y());
            mp_posis.push_back(temp_3d);
        }
        show_mp_as_cloud(mp_posis, "colmap_pts");
        Eigen::Matrix3Xd pt_from;
        pt_from.resize(3,8*reconstruction.NumRegImages());
        Eigen::Matrix3Xd pt_to;
        pt_to.resize(3,8*reconstruction.NumRegImages());
        int img_count=0;
        std::vector<visualization::Color> colors;
        colors.resize(8*reconstruction.NumRegImages());
        for(int i=0; i<colors.size(); i++){
            colors[i] = visualization::kCommonRed;
        }
        for(auto id: reconstruction.RegImageIds()){
            colmap::Image& image_t= reconstruction.Image(id);
            std::vector<Eigen::Vector3d> cam_verts_g;
            for(int i=0; i<cam_verts.size(); i++){
                Eigen::Vector3d temp=image_t.RotationMatrix().transpose()*cam_verts[i]+image_t.ProjectionCenter();
                Eigen::Vector3d temp2;
                temp2.x()=temp.x();
                temp2.y()=temp.z();
                temp2.z()=-temp.y();
                cam_verts_g.push_back(temp2);
            }
            std::cout<<image_t.Name()<<std::endl;
            pt_from.block(0, img_count*8+0,3,1)=cam_verts_g[0];
            pt_to.block(0, img_count*8+0,3,1)=cam_verts_g[1];
            pt_from.block(0, img_count*8+1,3,1)=cam_verts_g[0];
            pt_to.block(0, img_count*8+1,3,1)=cam_verts_g[2];
            pt_from.block(0, img_count*8+2,3,1)=cam_verts_g[0];
            pt_to.block(0, img_count*8+2,3,1)=cam_verts_g[3];
            pt_from.block(0, img_count*8+3,3,1)=cam_verts_g[0];
            pt_to.block(0, img_count*8+3,3,1)=cam_verts_g[4];
            
            pt_from.block(0, img_count*8+4,3,1)=cam_verts_g[1];
            pt_to.block(0, img_count*8+4,3,1)=cam_verts_g[2];
            pt_from.block(0, img_count*8+5,3,1)=cam_verts_g[2];
            pt_to.block(0, img_count*8+5,3,1)=cam_verts_g[3];
            pt_from.block(0, img_count*8+6,3,1)=cam_verts_g[3];
            pt_to.block(0, img_count*8+6,3,1)=cam_verts_g[4];
            pt_from.block(0, img_count*8+7,3,1)=cam_verts_g[4];
            pt_to.block(0, img_count*8+7,3,1)=cam_verts_g[1];
            //break;
            img_count++;
            
        }
        visualization::publishLines(pt_from, pt_to, colors, 1, 0.01, 1, "map", "cam", "cam_marker");
        ros::spin();
    }else if(FLAGS_op_type=="ply"){
        visualization::RVizVisualizationSink::init();
        std::vector<colmap::PlyPoint> points_colmap = colmap::ReadPly(FLAGS_workspace_path);
        Eigen::Matrix3Xd points;
        points.resize(3,points_colmap.size());
        std::vector<visualization::Color> colors;
        colors.resize(points_colmap.size());
        for(int i=0; i<points_colmap.size(); i++){
            points(0,i)=points_colmap[i].x;
            points(1,i)=points_colmap[i].z;
            points(2,i)=-points_colmap[i].y;
            colors[i].red=points_colmap[i].r;
            colors[i].green=points_colmap[i].g;
            colors[i].blue=points_colmap[i].b;
        } 
        publish3DPointsAsPointCloud(points, colors, 1.0, visualization::kDefaultMapFrame,"dense_color");
        ros::spin();
    }
    return 0;
}
