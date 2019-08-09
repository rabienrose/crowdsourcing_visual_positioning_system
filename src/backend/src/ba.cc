#include <string>
#include <fstream>
#include <memory>
#include <Eigen/Core>
#include <math.h>
#include <unordered_map>

#include "g2o/core/robust_kernel_impl.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/core/optimization_algorithm_gauss_newton.h"
#include "g2o/core/optimization_algorithm_with_hessian.h"
#include "g2o/solvers/linear_solver_dense.h"
#include "g2o/solvers/linear_solver_eigen.h"
#include "g2o/types/types_six_dof_expmap.h"
#include <backend/header.h>
#include "opencv2/opencv.hpp"
#include <glog/logging.h>
#include <gflags/gflags.h>

DECLARE_int32(opti_count);
DECLARE_double(gps_weight);
DEFINE_double(max_repro_err, 5, "");
DECLARE_double(t_c_g_x);
DECLARE_double(t_c_g_y);
DECLARE_double(t_c_g_z);

namespace g2o {   
    class EdgePosiPre : public BaseUnaryEdge<3, Eigen::Vector3d, VertexSE3Expmap>{
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        EdgePosiPre(){};

        bool read(std::istream& is){return true;};

        bool write(std::ostream& os) const{return true;};
        
        Eigen::Vector3d tcg=Eigen::Vector3d::Zero();

        void computeError()  {
        const g2o::VertexSE3Expmap* v1 = static_cast<const VertexSE3Expmap*>(_vertices[0]);
        _error= v1->estimate().inverse().rotation().toRotationMatrix()*tcg+v1->estimate().inverse().translation()-_measurement;
        }
    };
}

void optimize_BA(gm::GlobalMap& map, bool re_triangle){
    std::vector<cv::Mat> proj_cv_mats;
    std::unordered_map<long unsigned int, int> frameid_inds_map;
    for(int i=0; i<map.frames.size(); i++){
        Eigen::Matrix<double,3,4> P_double_34=map.frames[i]->getProjMat();
        cv::Mat cv_mat(3,4, CV_32FC1);
        for(int n=0; n<3; n++){
            for(int m=0; m<4; m++){
                cv_mat.at<float>(n,m)=P_double_34(n,m);
            }
        }
        frameid_inds_map[map.frames[i]->id]=i;
        proj_cv_mats.push_back(cv_mat);
    }

    if(re_triangle==true){
        for(int i=0; i<map.mappoints.size(); i++){
            if(map.mappoints[i]->track.size()>=2){
                int last_id=map.mappoints[i]->track.size()-1;
                float u,v;
                int octave;
                map.mappoints[i]->track[0].getUV(u, v, octave);
                std::vector<cv::Point2f> pts1;
                cv::Point2f pt1(u, v);
                pts1.push_back(pt1);
                std::vector<cv::Point2f> pts2;
                map.mappoints[i]->track[last_id].getUV(u, v, octave);
                cv::Point2f pt2( u, v);
                pts2.push_back(pt2);
                cv::Mat proj1=proj_cv_mats[frameid_inds_map[map.mappoints[i]->track[0].frame->id]];
                cv::Mat proj2=proj_cv_mats[frameid_inds_map[map.mappoints[i]->track[last_id].frame->id]];
                cv::Mat out_posi;
                cv::triangulatePoints(proj1, proj2, pts1, pts2, out_posi);
                Eigen::Vector3d temp_posi;
                temp_posi(0)=out_posi.at<float>(0)/out_posi.at<float>(3);
                temp_posi(1)=out_posi.at<float>(1)/out_posi.at<float>(3);
                temp_posi(2)=out_posi.at<float>(2)/out_posi.at<float>(3);
                map.mappoints[i]->position=temp_posi;
                //std::cout<<mp_posis_out[i].transpose()<<std::endl;
            }else{
                map.mappoints[i]->isbad=true;
            }
        }
    }else{
        for(int i=0; i<map.mappoints.size(); i++){
            if(map.mappoints[i]->track.size()>=2){
            }else{
                map.mappoints[i]->isbad=true;
            }
        }
    }
    
    int nlevels=8;
    float scaleFactor=1.2;
    std::vector<float> mvScaleFactor;
    std::vector<float> mvInvScaleFactor;    
    std::vector<float> mvLevelSigma2;
    std::vector<float> mvInvLevelSigma2;
    mvScaleFactor.resize(nlevels);
    mvLevelSigma2.resize(nlevels);
    mvScaleFactor[0]=1.0f;
    mvLevelSigma2[0]=1.0f;
    for(int i=1; i<nlevels; i++)
    {
        mvScaleFactor[i]=mvScaleFactor[i-1]*scaleFactor;
        mvLevelSigma2[i]=mvScaleFactor[i]*mvScaleFactor[i];
    }

    mvInvScaleFactor.resize(nlevels);
    mvInvLevelSigma2.resize(nlevels);
    for(int i=0; i<nlevels; i++)
    {
        mvInvScaleFactor[i]=1.0f/mvScaleFactor[i];
        mvInvLevelSigma2[i]=1.0f/mvLevelSigma2[i];
    }
    g2o::SparseOptimizer optimizer;
    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(
        new g2o::BlockSolverX(
            new g2o::LinearSolverEigen<g2o::BlockSolverX::PoseMatrixType>()));
    optimizer.setAlgorithm(solver);
    
    //add all vertice, number equal all poses 
    std::vector<g2o::VertexSE3Expmap*> kf_verts;
    long unsigned int maxKFid = 0;
    std::unordered_map<long unsigned int, int> frame_vertex_map;
    std::vector<std::shared_ptr<gm::Frame>> v_2_frames;
    for(int i=0; i<map.frames.size(); i++){
        g2o::VertexSE3Expmap* vSE3 = new g2o::VertexSE3Expmap();
        Eigen::Matrix<double,3,3> R(map.frames[i]->direction);
        Eigen::Matrix<double,3,1> t=map.frames[i]->position;
        vSE3->setEstimate(g2o::SE3Quat(R,t).inverse());
        vSE3->setId(i);
        vSE3->setFixed(false);
        kf_verts.push_back(vSE3);
        v_2_frames.push_back(map.frames[i]);
        frame_vertex_map[map.frames[i]->id]=kf_verts.size()-1;
        optimizer.addVertex(vSE3);
        if(i>maxKFid)
            maxKFid=i;
    }
    std::cout<<"add vertice"<<std::endl;
    
    //add gps edge, leave vertice without gps empty 
    std::vector<g2o::EdgePosiPre*> lidar_edges;
    for(int i=0; i<map.frames.size(); i++){
        if(map.frames[i]->gps_accu<30){
            g2o::EdgePosiPre* e = new g2o::EdgePosiPre();
            e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(kf_verts[frame_vertex_map[map.frames[i]->id]]));
            e->setMeasurement(map.frames[i]->gps_position);
            e->setInformation(Eigen::Matrix<double, 3, 3>::Identity()*FLAGS_gps_weight);
            e->tcg(0)=FLAGS_t_c_g_x;
            e->tcg(1)=FLAGS_t_c_g_y;
            e->tcg(2)=FLAGS_t_c_g_z;
            optimizer.addEdge(e);
            lidar_edges.push_back(e);
        }
    }
    std::cout<<"add lidar edge"<<std::endl;
    
    const float thHuber2D = sqrt(5.99);
    const float thHuber3D = sqrt(7.815);
    
    //add projection edge, number equal to mappoints*track
    //add mappoint vertice
    std::vector<g2o::VertexSBAPointXYZ*> mp_verts;
    std::vector<g2o::EdgeSE3ProjectXYZ*> proj_edges;
    for(int i=0; i<map.mappoints.size(); i++){
        if(map.mappoints[i]->isbad==true){
            mp_verts.push_back(0);
            continue;
        }
        for(int j=0; j<map.mappoints[i]->track.size(); j++){
            if(j==0){
                g2o::VertexSBAPointXYZ* vPoint = new g2o::VertexSBAPointXYZ();
                vPoint->setEstimate(map.mappoints[i]->position);
                vPoint->setMarginalized(true);
                const int id = i+maxKFid+1+2;
                vPoint->setId(id);
                optimizer.addVertex(vPoint);
                mp_verts.push_back(vPoint);
            }
            Eigen::Matrix<double,2,1> obs;
            float u,v;
            int octave;
            map.mappoints[i]->track[j].getUV(u, v, octave);
            obs << u, v;

            g2o::EdgeSE3ProjectXYZ* e = new g2o::EdgeSE3ProjectXYZ();
            std::shared_ptr<gm::Frame> mp_frame_p=map.mappoints[i]->track[j].frame;

            e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(mp_verts.back()));
            e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(kf_verts[frame_vertex_map[mp_frame_p->id]]));
            
            e->setMeasurement(obs);
            const float &invSigma2 = mvInvLevelSigma2[octave];
            e->setInformation(Eigen::Matrix2d::Identity()*invSigma2);

            g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
            e->setRobustKernel(rk);
            rk->setDelta(thHuber2D);
            e->fx = mp_frame_p->fx;
            e->fy = mp_frame_p->fy;
            e->cx = mp_frame_p->cx;
            e->cy = mp_frame_p->cy;

            optimizer.addEdge(e);
            
            if(true){
                //std::cout<<mp_verts.back()->estimate().transpose()<<std::endl;
                e->computeError();
                //std::cout<<e->chi2()<<std::endl;
            }
            proj_edges.push_back(e);
        }
    }
//         std::cout<<"add project edge"<<std::endl;
    
    float avg_error=0;
    for(int i=0; i<lidar_edges.size(); i++){
        lidar_edges[i]->computeError();
        avg_error=avg_error+sqrt(lidar_edges[i]->chi2())/lidar_edges.size();
    }
    std::cout<<"lidar edge err before: "<<avg_error<<std::endl;
    avg_error=0;
    for(int i=0; i<proj_edges.size(); i++){
        proj_edges[i]->computeError();
        avg_error=avg_error+sqrt(proj_edges[i]->chi2())/proj_edges.size();
        //std::cout<<"avg_error: "<<avg_error<<"||"<<sqrt(proj_edges[i]->chi2())<<std::endl;
        if(avg_error<0){
            return;
        }
        
    }
    std::cout<<"project edge err before: "<<avg_error<<std::endl;
    clock_t time;
    time = clock();
    optimizer.initializeOptimization();
    optimizer.optimize(FLAGS_opti_count);
    time = clock() - time;
    //std::cout<<"cam after: "<<vCam->estimate().transpose()<<std::endl;
    std::cout<<"opt time: "<<((float)time)/CLOCKS_PER_SEC<<std::endl;
    avg_error=0;
    for(int i=0; i<lidar_edges.size(); i++){
        lidar_edges[i]->computeError();
        avg_error=avg_error+sqrt(lidar_edges[i]->chi2())/lidar_edges.size();
    }
    std::cout<<"lidar edge err after: "<<avg_error<<std::endl;
    avg_error=0;
    for(int i=0; i<proj_edges.size(); i++){
        proj_edges[i]->computeError();
        avg_error=avg_error+sqrt(proj_edges[i]->chi2())/proj_edges.size();
    }
    std::cout<<"project edge err after: "<<avg_error<<std::endl;
    
    for(int i=0; i<mp_verts.size(); i++){
        if(mp_verts[i]!=0){
            map.mappoints[i]->position=mp_verts[i]->estimate();
        }
    }
    for(int i=0; i<kf_verts.size(); i++){
        v_2_frames[i]->setPose(kf_verts[i]->estimate().inverse().to_homogeneous_matrix());
    }
    int del_edge_count=0;
    for(int i=0; i<map.frames.size(); i++){
        for(int j=0; j<map.frames[i]->obss.size(); j++){
            if(map.frames[i]->obss[j]!=nullptr){
                Eigen::Matrix<double, 3,4> proj_mat = map.frames[i]->getProjMat();
                Eigen::Vector4d posi_homo;
                posi_homo.block(0,0,3,1)=map.frames[i]->obss[j]->position;
                posi_homo(3)=1;
                Eigen::Vector3d proj_homo = proj_mat*posi_homo;
                //std::cout<<proj_mat<<std::endl;
                double u=proj_homo(0)/proj_homo(2);
                double v=proj_homo(1)/proj_homo(2);
                cv::Point2f uv= map.frames[i]->kps[j].pt;
                //std::cout<<u<<":"<<v<<"     "<<uv.x<<":"<<uv.y<<std::endl;
                
                float proj_err=sqrt((uv.x-u)*(uv.x-u)+(uv.y-v)*(uv.y-v));
                if(proj_err>FLAGS_max_repro_err || proj_homo(2)<0){
                    del_edge_count++;
                    map.frames[i]->obss[j]=nullptr;
                }
            }
        }
    }
    int mp_count=0;
    map.AssignKpToMp();
    int total_size=map.mappoints.size();
    for(int i=total_size-1; i>=0; i--){
        if(map.mappoints[i]->track.size()<2){
            map.DelMappoint(map.mappoints[i]->id);
            mp_count++;
        }
    }
}