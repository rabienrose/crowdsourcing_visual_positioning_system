#include <backend/header.h>

#include "g2o/core/robust_kernel_impl.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/solvers/linear_solver_eigen.h"
#include "g2o/types/types_seven_dof_expmap.h"
#include "opencv2/opencv.hpp"
#include <glog/logging.h>
#include <gflags/gflags.h>

#ifdef VISUALIZATION
#include "visualization/color-palette.h"
#include "visualization/color.h"
#include "visualization/common-rviz-visualization.h"
#endif


DEFINE_int32(opti_count, 100, "How many of the iteration of optimization");
DEFINE_double(gps_weight, 0.01, "The weight of GPS impact in optimization");


namespace g2o {
    class EdgePosiPreSim3 : public BaseUnaryEdge<3, Eigen::Vector3d, VertexSim3Expmap>{
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        EdgePosiPreSim3(){};

        bool read(std::istream& is){return true;};

        bool write(std::ostream& os) const{return true;};

        void computeError()  {
        const g2o::VertexSim3Expmap* v1 = static_cast<const VertexSim3Expmap*>(_vertices[0]);
        _error= v1->estimate().inverse().translation()-_measurement;
        //std::cout<<v1->estimate().inverse().translation().transpose()<<std::endl;
        //std::cout<<v1->estimate().scale()<<std::endl;
        //std::cout<<v1->estimate().inverse().translation().transpose()/v1->estimate().scale()<<std::endl;
        }
    };
    class EdgePosiPreSE3 : public BaseUnaryEdge<3, Eigen::Vector3d, VertexSE3Expmap>{
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        EdgePosiPreSE3(){};

        bool read(std::istream& is){return true;};

        bool write(std::ostream& os) const{return true;};

        void computeError()  {
        const g2o::VertexSE3Expmap* v1 = static_cast<const VertexSE3Expmap*>(_vertices[0]);
        _error= v1->estimate().inverse().translation()-_measurement;
        }
    };
    class EdgeSE3 : public BaseBinaryEdge<6, g2o::SE3Quat, VertexSE3Expmap, VertexSE3Expmap>{
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        EdgeSE3(){};

        bool read(std::istream& is){return true;};

        bool write(std::ostream& os) const{return true;};

        void computeError()  {
            const VertexSE3Expmap* v1 = static_cast<const VertexSE3Expmap*>(_vertices[0]);
            const VertexSE3Expmap* v2 = static_cast<const VertexSE3Expmap*>(_vertices[1]);

            g2o::SE3Quat C(_measurement);
            g2o::SE3Quat error_=C*v1->estimate()*v2->estimate().inverse();
            _error = error_.log();
        }
    };
}

// void show_mp_as_cloud(std::vector<Eigen::Vector3d>& mp_posis, std::string topic){
//     Eigen::Matrix3Xd points;
//     points.resize(3,mp_posis.size());
//     for(int i=0; i<mp_posis.size(); i++){
//         points.block<3,1>(0,i)=mp_posis[i];
//     }    
//     publish3DPointsAsPointCloud(points, visualization::kCommonRed, 1.0, visualization::kDefaultMapFrame,topic);
// }

void pose_graph_opti_sim3(gm::GlobalMap& map){
//    g2o::SparseOptimizer optimizer;
//    optimizer.setVerbose(false);
//
//    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(
//        new g2o::BlockSolver_7_3(
//            new g2o::LinearSolverEigen<g2o::BlockSolver_7_3::PoseMatrixType>()));
//
//    solver->setUserLambdaInit(1e-16);
//    optimizer.setAlgorithm(solver);
//
//    std::vector<g2o::VertexSim3Expmap*> v_sim3_list;
//    std::vector<g2o::Sim3> sim3_list;
//
//    std::map<std::shared_ptr<gm::Frame>, g2o::VertexSim3Expmap*> frame_to_vertex;
//    std::map<g2o::VertexSim3Expmap*, std::shared_ptr<gm::Frame>> vertex_to_frame;
////     std::vector<Eigen::Vector3d> debug_points;
////     for(int n=0; n<map.frames.size(); n++){
////         if(map.frames[n]->isborder==true){
////             debug_points.push_back(map.frames[n]->position);
////         }
////     }
////     show_mp_as_cloud(debug_points, "debug1");
////     ros::spin();
//
//    for(int i=0; i<map.frames.size(); i++){
//        g2o::VertexSim3Expmap* VSim3 = new g2o::VertexSim3Expmap();
//        Eigen::Matrix4d pose_temp = map.frames[i]->getPose();
//        Eigen::Matrix4d pose_inv=pose_temp.inverse();
//
//        Eigen::Matrix<double,3,3> Rcw = pose_inv.block(0,0,3,3);
//        Eigen::Matrix<double,3,1> tcw = pose_inv.block(0,3,3,1);
//        g2o::Sim3 Siw;
//        double scale=Rcw.block(0,0,3,1).norm();
//        Rcw=Rcw/scale;
//        Siw=g2o::Sim3(Rcw,tcw,scale);
//
//        VSim3->setEstimate(Siw);
//        VSim3->setFixed(map.frames[i]->isborder);
//
//        VSim3->setId(i);
//        VSim3->setMarginalized(false);
//
//        optimizer.addVertex(VSim3);
//        v_sim3_list.push_back(VSim3);
//        sim3_list.push_back(Siw);
//        frame_to_vertex[map.frames[i]]=VSim3;
//        vertex_to_frame[VSim3]=map.frames[i];
//    }
//
//    std::vector<g2o::EdgePosiPreSim3*> gps_edges;
//    for(int i=0; i<map.frames.size(); i++){
//        if(map.frames[i]->isborder==true){
//            continue;
//        }
//        if(map.frames[i]->gps_accu<30){
//            g2o::EdgePosiPreSim3* e = new g2o::EdgePosiPreSim3();
//            //std::cout<<v_sim3_list[i]->estimate().inverse().translation().transpose()<<std::endl;
//            e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(frame_to_vertex[map.frames[i]]));
//            e->setMeasurement(map.frames[i]->gps_position);
//            Eigen::Matrix<double, 3, 3> con_mat = Eigen::Matrix<double, 3, 3>::Identity()/map.frames[i]->gps_accu*FLAGS_gps_weight;
//            con_mat(2,2)=0.0000001;
//            e->information()=con_mat;
//            optimizer.addEdge(e);
//            gps_edges.push_back(e);
//            //e->computeError();
//        }
//    }
//    std::cout<<"add gps edge: "<<gps_edges.size()<<std::endl;
//
//    std::vector<g2o::EdgeSim3*> sim3_edge_list;
//    for(int i=0; i<map.pose_graph_v1.size(); i++){
//        //std::cout<<Eigen::Matrix3d(map.pose_graph_e_rot[i])<<std::endl;
//        //std::cout<<map.pose_graph_e_posi[i].transpose()<<std::endl;
//        g2o::Sim3 Sji(map.pose_graph_e_rot[i],map.pose_graph_e_posi[i],map.pose_graph_e_scale[i]);
//        g2o::EdgeSim3* e = new g2o::EdgeSim3();
//        //std::cout<<"v1: "<<frame_to_vertex[map.pose_graph_v1[i]]->estimate().inverse().translation().transpose()<<std::endl;
//        //std::cout<<"obs: "<<(frame_to_vertex[map.pose_graph_v2[i]]->estimate().inverse().rotation().toRotationMatrix()*Sji.translation()+frame_to_vertex[map.pose_graph_v2[i]]->estimate().inverse().translation()).transpose()<<std::endl;
//        e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(frame_to_vertex[map.pose_graph_v1[i]]));
//        e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(frame_to_vertex[map.pose_graph_v2[i]]));
//        e->setMeasurement(Sji);
//        //std::cout<<map.pose_graph_weight[i]<<std::endl;
//        if(map.pose_graph_weight[i]<1){
//            map.pose_graph_weight[i]=100;
//        }
//        Eigen::Matrix<double,7,7> matLambda = Eigen::Matrix<double,7,7>::Identity()*map.pose_graph_weight[i];
//        e->information() = matLambda;
//        e->computeError();
//
//        optimizer.addEdge(e);
//        sim3_edge_list.push_back(e);
//    }
//    std::cout<<"add sim3 edge: "<<sim3_edge_list.size()<<std::endl;
//
//    float avg_error=0;
//    for(int i=0; i<gps_edges.size(); i++){
//        gps_edges[i]->computeError();
//        avg_error=avg_error+sqrt(gps_edges[i]->chi2())/gps_edges.size();
//    }
//    std::cout<<"gps edge err before: "<<avg_error<<std::endl;
//    avg_error=0;
//    for(int i=0; i<sim3_edge_list.size(); i++){
//        sim3_edge_list[i]->computeError();
//        avg_error=avg_error+sqrt(sim3_edge_list[i]->chi2())/sim3_edge_list.size();
////             if(sqrt(sim3_edge_list[i]->chi2())>1){
////                 std::cout<<"avg_error: "<<avg_error<<"||"<<sqrt(sim3_edge_list[i]->chi2())<<std::endl;
////             }
//        if(avg_error<0){
//            return;
//        }
//    }
//    std::cout<<"sim3 edge err before: "<<avg_error<<std::endl;
//
//    optimizer.initializeOptimization();
//    //optimizer.computeInitialGuess();
//    optimizer.optimize(FLAGS_opti_count);
//
//    avg_error=0;
//    for(int i=0; i<gps_edges.size(); i++){
//        gps_edges[i]->computeError();
//        avg_error=avg_error+sqrt(gps_edges[i]->chi2())/gps_edges.size();
//    }
//    std::cout<<"gps edge err after: "<<avg_error<<std::endl;
//    avg_error=0;
//    for(int i=0; i<sim3_edge_list.size(); i++){
//        sim3_edge_list[i]->computeError();
//        avg_error=avg_error+sqrt(sim3_edge_list[i]->chi2())/sim3_edge_list.size();
//        if(avg_error<0){
//            return;
//        }
//
//    }
//    std::cout<<"sim3 edge err after: "<<avg_error<<std::endl;
//    for(int i=0; i<sim3_list.size(); i++){
//        g2o::Sim3 CorrectedSiw =  v_sim3_list[i]->estimate();
//        Eigen::Matrix3d eigR = CorrectedSiw.rotation().toRotationMatrix();
//        Eigen::Vector3d eigt = CorrectedSiw.translation();
//        double s = CorrectedSiw.scale();
//        Eigen::Matrix4d Tiw=Eigen::Matrix4d::Identity();
//        eigt *=(1./s); //[R t/s;0 1]
//        Tiw.block(0,0,3,3)=eigR;
//        Tiw.block(0,3,3,1)=eigt;
//        vertex_to_frame[v_sim3_list[i]]->setPose(Tiw.inverse());
//    }
}

void pose_graph_opti_se3(gm::GlobalMap& map){
    g2o::SparseOptimizer optimizer;
    optimizer.setVerbose(false);
    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(
        new g2o::BlockSolverX(
            new g2o::LinearSolverEigen<g2o::BlockSolverX::PoseMatrixType>()));

    solver->setUserLambdaInit(1e-16);
    optimizer.setAlgorithm(solver);
    
    std::vector<g2o::VertexSE3Expmap*> v_se3_list;
    
    std::map<std::shared_ptr<gm::Frame>, g2o::VertexSE3Expmap*> frame_to_vertex;
    std::map<g2o::VertexSE3Expmap*, std::shared_ptr<gm::Frame>> vertex_to_frame;
//     std::vector<Eigen::Vector3d> debug_points;
//     for(int n=0; n<map.frames.size(); n++){
//         if(map.frames[n]->isborder==true){
//             debug_points.push_back(map.frames[n]->position);
//         }
//     }
//     show_mp_as_cloud(debug_points, "debug1");
//     ros::spin();
    bool any_fix_frame=false;
    for(int i=0; i<map.frames.size(); i++){
        g2o::VertexSE3Expmap* vSE3 = new g2o::VertexSE3Expmap();
        Eigen::Matrix<double,3,3> R(map.frames[i]->direction);
        Eigen::Matrix<double,3,1> t=map.frames[i]->position;
        vSE3->setEstimate(g2o::SE3Quat(R,t).inverse());
        vSE3->setFixed(map.frames[i]->isborder);
        if(map.frames[i]->isborder==true){
            any_fix_frame=true;
        }
        
        vSE3->setId(i);
        vSE3->setMarginalized(false);
        
        optimizer.addVertex(vSE3);
        v_se3_list.push_back(vSE3);
        frame_to_vertex[map.frames[i]]=vSE3;
        vertex_to_frame[vSE3]=map.frames[i];
    }
    if(any_fix_frame==false){
        if(v_se3_list.size()>0){
            v_se3_list[0]->setFixed(true);
        }
    }
    std::vector<g2o::EdgePosiPreSE3*> gps_edges;
    for(int i=0; i<map.frames.size(); i++){
        if(map.frames[i]->isborder==true){
            continue;
        }
        if(map.frames[i]->gps_accu<30){
            g2o::EdgePosiPreSE3* e = new g2o::EdgePosiPreSE3();
            e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(frame_to_vertex[map.frames[i]]));
            e->setMeasurement(map.frames[i]->gps_position);
            Eigen::Matrix<double, 3, 3> con_mat = Eigen::Matrix<double, 3, 3>::Identity()/map.frames[i]->gps_accu*FLAGS_gps_weight;
            con_mat(2,2)=0.0000001;
            e->information()=con_mat;
            optimizer.addEdge(e);
            gps_edges.push_back(e);
            //e->computeError();
        }
    }
    std::cout<<"add gps edge: "<<gps_edges.size()<<std::endl;
    
    std::vector<g2o::EdgeSE3*> se3_edge_list;
    for(int i=0; i<map.pose_graph_v1.size(); i++){
        //std::cout<<Eigen::Matrix3d(map.pose_graph_e_rot[i])<<std::endl;
        //std::cout<<map.pose_graph_e_posi[i].transpose()<<std::endl;
        g2o::SE3Quat Sji(map.pose_graph_e_rot[i],map.pose_graph_e_posi[i]);
        g2o::EdgeSE3* e = new g2o::EdgeSE3();
        //std::cout<<"v1: "<<frame_to_vertex[map.pose_graph_v1[i]]->estimate().inverse().translation().transpose()<<std::endl;
        //std::cout<<"obs: "<<(frame_to_vertex[map.pose_graph_v2[i]]->estimate().inverse().rotation().toRotationMatrix()*Sji.translation()+frame_to_vertex[map.pose_graph_v2[i]]->estimate().inverse().translation()).transpose()<<std::endl;
        e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(frame_to_vertex[map.pose_graph_v1[i]]));
        e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(frame_to_vertex[map.pose_graph_v2[i]]));
        e->setMeasurement(Sji);
        //std::cout<<map.pose_graph_weight[i]<<std::endl;
        if(map.pose_graph_weight[i]<1){
            map.pose_graph_weight[i]=100;
        }
        Eigen::Matrix<double,6,6> matLambda = Eigen::Matrix<double,6,6>::Identity()*map.pose_graph_weight[i];
        //Eigen::Matrix<double,6,6> matLambda = Eigen::Matrix<double,6,6>::Identity();
        e->information() = matLambda;
        e->computeError();

        optimizer.addEdge(e);
        se3_edge_list.push_back(e);
    }
    std::cout<<"add sim3 edge: "<<se3_edge_list.size()<<std::endl;
    
    float avg_error=0;
    for(int i=0; i<gps_edges.size(); i++){
        gps_edges[i]->computeError();
        avg_error=avg_error+sqrt(gps_edges[i]->chi2())/gps_edges.size();
    }
    std::cout<<"gps edge err before: "<<avg_error<<std::endl;
    avg_error=0;
    for(int i=0; i<se3_edge_list.size(); i++){
        se3_edge_list[i]->computeError();
        avg_error=avg_error+sqrt(se3_edge_list[i]->chi2())/se3_edge_list.size();
        if(avg_error<0){
            return;
        }
    }
    std::cout<<"sim3 edge err before: "<<avg_error<<std::endl;
    
    optimizer.initializeOptimization();
    optimizer.computeInitialGuess();
    optimizer.optimize(FLAGS_opti_count);
    
    avg_error=0;
    for(int i=0; i<gps_edges.size(); i++){
        gps_edges[i]->computeError();
        avg_error=avg_error+sqrt(gps_edges[i]->chi2())/gps_edges.size();
    }
    std::cout<<"gps edge err after: "<<avg_error<<std::endl;
    avg_error=0;
    for(int i=0; i<se3_edge_list.size(); i++){
        se3_edge_list[i]->computeError();
        avg_error=avg_error+sqrt(se3_edge_list[i]->chi2())/se3_edge_list.size();
        if(avg_error<0){
            return;
        }
        
    }
    std::cout<<"sim3 edge err after: "<<avg_error<<std::endl;
    for(int i=0; i<v_se3_list.size(); i++){
        g2o::SE3Quat CorrectedSiw =  v_se3_list[i]->estimate();
        Eigen::Matrix3d eigR = CorrectedSiw.rotation().toRotationMatrix();
        Eigen::Vector3d eigt = CorrectedSiw.translation();
        Eigen::Matrix4d Tiw=Eigen::Matrix4d::Identity();
        Tiw.block(0,0,3,3)=eigR;
        Tiw.block(0,3,3,1)=eigt;
        vertex_to_frame[v_se3_list[i]]->setPose(Tiw.inverse());
    }
}
