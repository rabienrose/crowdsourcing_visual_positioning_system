#include "sim3_ransac/sim3_match.h"
#include <Eigen/Dense>
#include <vector>
#include "g2o/core/block_solver.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/solvers/linear_solver_eigen.h"
#include "g2o/types/types_six_dof_expmap.h"
#include "g2o/core/robust_kernel_impl.h"
#include "g2o/solvers/linear_solver_dense.h"
#include "g2o/types/types_seven_dof_expmap.h"
#include <glog/logging.h>
#include <cmath>
namespace chamo
{
    
    void ComputeCentroid(cv::Mat &P, cv::Mat &Pr, cv::Mat &C)
    {
        cv::reduce(P,C,1,cv::REDUCE_SUM);
        C = C/P.cols;

        for(int i=0; i<P.cols; i++)
        {
            Pr.col(i)=P.col(i)-C;
        }
    }
    
    void ComputeSim3(cv::Mat &P1, cv::Mat &P2, cv::Mat& mT12i, double& scale)
    {
        cv::Mat mR12i;
        cv::Mat mt12i;
        float ms12i;
        std::vector<bool> mvbInliersi;
        int mnInliersi;

        cv::Mat Pr1(P1.size(),P1.type()); // Relative coordinates to centroid (set 1)
        cv::Mat Pr2(P2.size(),P2.type()); // Relative coordinates to centroid (set 2)
        cv::Mat O1(3,1,Pr1.type()); // Centroid of P1
        cv::Mat O2(3,1,Pr2.type()); // Centroid of P2

        ComputeCentroid(P1,Pr1,O1);
        ComputeCentroid(P2,Pr2,O2);

        // Step 2: Compute M matrix

        cv::Mat M = Pr2*Pr1.t();

        // Step 3: Compute N matrix

        double N11, N12, N13, N14, N22, N23, N24, N33, N34, N44;

        cv::Mat N(4,4,P1.type());

        N11 = M.at<float>(0,0)+M.at<float>(1,1)+M.at<float>(2,2);
        N12 = M.at<float>(1,2)-M.at<float>(2,1);
        N13 = M.at<float>(2,0)-M.at<float>(0,2);
        N14 = M.at<float>(0,1)-M.at<float>(1,0);
        N22 = M.at<float>(0,0)-M.at<float>(1,1)-M.at<float>(2,2);
        N23 = M.at<float>(0,1)+M.at<float>(1,0);
        N24 = M.at<float>(2,0)+M.at<float>(0,2);
        N33 = -M.at<float>(0,0)+M.at<float>(1,1)-M.at<float>(2,2);
        N34 = M.at<float>(1,2)+M.at<float>(2,1);
        N44 = -M.at<float>(0,0)-M.at<float>(1,1)+M.at<float>(2,2);

        N = (cv::Mat_<float>(4,4) << N11, N12, N13, N14,
                                    N12, N22, N23, N24,
                                    N13, N23, N33, N34,
                                    N14, N24, N34, N44);


        // Step 4: Eigenvector of the highest eigenvalue

        cv::Mat eval, evec;

        cv::eigen(N,eval,evec); //evec[0] is the quaternion of the desired rotation

        cv::Mat vec(1,3,evec.type());
        (evec.row(0).colRange(1,4)).copyTo(vec); //extract imaginary part of the quaternion (sin*axis)

        // Rotation angle. sin is the norm of the imaginary part, cos is the real part
        double ang=atan2(norm(vec),evec.at<float>(0,0));

        vec = 2*ang*vec/norm(vec); //Angle-axis representation. quaternion angle is the half

        mR12i.create(3,3,P1.type());

        cv::Rodrigues(vec,mR12i); // computes the rotation matrix from angle-axis
        // Step 5: Rotate set 2

        cv::Mat P3 = mR12i*Pr2;
        

        // Step 6: Scale

        double nom = Pr1.dot(P3);
        cv::Mat aux_P3(P3.size(),P3.type());
        aux_P3=P3;
        cv::pow(P3,2,aux_P3);
        double den = 0;

        for(int i=0; i<aux_P3.rows; i++)
        {
            for(int j=0; j<aux_P3.cols; j++)
            {
                den+=aux_P3.at<float>(i,j);
            }
        }
        ms12i = nom/den;

        // Step 7: Translation

        mt12i.create(1,3,P1.type());
        mt12i = O1 - ms12i*mR12i*O2;

        // Step 8: Transformation

        // Step 8.1 T12
        mT12i = cv::Mat::eye(4,4,P1.type());

        cv::Mat sR = ms12i*mR12i;
        scale=ms12i;
        sR.copyTo(mT12i.rowRange(0,3).colRange(0,3));
        mt12i.copyTo(mT12i.rowRange(0,3).col(3));
    }
    
    int OptimizeSim3Align(std::vector<Eigen::Vector3d>& global_mp,
                                 std::vector<Eigen::Vector3d>& local_mp,
                                 Eigen::Matrix4d& T12,
                                 double &scale_12,
                                 const float th2)
    {
        Eigen::Matrix3d R = T12.block(0,0,3,3) / scale_12;
        Eigen::Vector3d T = T12.block(0,3,3,1);
        // std::cout<<T12<<std::endl;
        // std::cout<<"scale: "<<scale_12<<std::endl;
        g2o::Sim3 g2oS12(R, T, scale_12);
        g2o::SparseOptimizer optimizer;
        
        g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(
            new g2o::BlockSolverX(
                new g2o::LinearSolverDense<g2o::BlockSolverX::PoseMatrixType>()));
        optimizer.setAlgorithm(solver);

        // Set Sim3 vertex
        g2o::VertexSim3Expmap * vSim3 = new g2o::VertexSim3Expmap();    
        vSim3->_fix_scale= false;
        vSim3->setEstimate(g2oS12);
        vSim3->setId(0);
        vSim3->setFixed(false);

        optimizer.addVertex(vSim3);

        // Set MapPoint vertices
        const int N = global_mp.size();
        std::vector<g2o::EdgeSim3XYZ*> vpEdges12;

        std::vector<size_t> vnIndexEdge;

        vnIndexEdge.reserve(2*N);
        vpEdges12.reserve(2*N);

        const float deltaHuber = sqrt(th2);

        int nCorrespondences = 0;

        float invSigmaSquare2 = 1 / 0.5; 
        for(int i=0; i<N; i++)
        {
            const int id1 = i+1;

            //add vertex
            g2o::VertexSBAPointXYZ* vPoint1 = new g2o::VertexSBAPointXYZ();
            vPoint1->setEstimate(local_mp[i]);
            vPoint1->setId(id1);
            vPoint1->setFixed(true);
            optimizer.addVertex(vPoint1);

            nCorrespondences++;

            // Set edge x2 = S21*X1
            Eigen::Matrix<double,3,1> obs;
            obs << global_mp[i][0],global_mp[i][1],global_mp[i][2];

            g2o::EdgeSim3XYZ* e12 = new g2o::EdgeSim3XYZ();

            e12->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id1)));
            e12->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(0)));
            e12->setMeasurement(obs);
            e12->setInformation(Eigen::Matrix3d::Identity()*invSigmaSquare2);
            g2o::RobustKernelHuber* rk2 = new g2o::RobustKernelHuber;
            e12->setRobustKernel(rk2);
            rk2->setDelta(deltaHuber);
            optimizer.addEdge(e12);

            vpEdges12.push_back(e12);
            vnIndexEdge.push_back(i);
        }

        // Optimize!
        optimizer.initializeOptimization();
        optimizer.optimize(5);

        // Check inliers
        int nBad=0;
        for(size_t i=0; i<vpEdges12.size();i++)
        {
            g2o::EdgeSim3XYZ* e12 = vpEdges12[i];
            if(!e12)
                continue;

            // std::cout<<"chi: "<<e12->chi2()<<std::endl;
            if(e12->chi2()>th2)
            {
                size_t idx = vnIndexEdge[i];
                optimizer.removeEdge(e12);
                vpEdges12[i]=static_cast<g2o::EdgeSim3XYZ*>(NULL);
                nBad++;
            }
        }
        // std::cout<<"nBad: "<<nBad<<std::endl;
        int nMoreIterations;
        if(nBad>0)
            nMoreIterations=10;
        else
            nMoreIterations=5;

        if(nCorrespondences-nBad<10)
            return 0;

        // Optimize again only with inliers

        optimizer.initializeOptimization();
        optimizer.optimize(nMoreIterations);

        int nIn = 0;
        for(size_t i=0; i<vpEdges12.size();i++)
        {
            g2o::EdgeSim3XYZ* e12 = vpEdges12[i];
            if(!e12)
                continue;

            if(e12->chi2() <= th2)
            {
                nIn++;
            }
        }

        // Recover optimized Sim3
        g2o::VertexSim3Expmap* vSim3_recov = static_cast<g2o::VertexSim3Expmap*>(optimizer.vertex(0));
        g2oS12= vSim3_recov->estimate();
    
        T12.block(0,0,3,3) = g2oS12.rotation().toRotationMatrix() * g2oS12.scale();
        T12.block(0,3,3,1) = g2oS12.translation();
        scale_12 = g2oS12.scale();
        // std::cout<<"nIn: "<<nIn<<std::endl;
        // std::cout<<T12<<std::endl;
        // std::cout<<"scale: "<<scale_12<<std::endl;
        return nIn;
    }
    
    void TransformPositionUseSim3(const Eigen::Matrix4d& sim3,
                                                  const double scale,
                                                  Eigen::Vector3d& in_position,
                                                  Eigen::Vector3d& out_position)
    {
        Eigen::Vector4d in_tmp(in_position(0), in_position(1), in_position(2), 1);
        Eigen::Vector4d out_tmp = sim3 * in_tmp;
        out_position(0) = out_tmp(0);
        out_position(1) = out_tmp(1);
        out_position(2) = out_tmp(2);
    }
    
    void ComputeSim3(std::vector<Eigen::Vector3d>& P1, std::vector<Eigen::Vector3d>& P2, Eigen::Matrix4d& T12i_eig, double& scale){
        cv::Mat p1mat(3, P1.size(), CV_32FC1);
        cv::Mat p2mat(3, P2.size(), CV_32FC1);
        for(int i=0; i<P1.size(); i++){
            p1mat.at<float>(0, i)=P1[i](0);
            p1mat.at<float>(1, i)=P1[i](1);
            p1mat.at<float>(2, i)=P1[i](2);
            p2mat.at<float>(0, i)=P2[i](0);
            p2mat.at<float>(1, i)=P2[i](1);
            p2mat.at<float>(2, i)=P2[i](2);
        }
        cv::Mat pose_mat;
        ComputeSim3(p1mat, p2mat, pose_mat, scale);
        for(int i=0; i<4; i++){
            for(int j=0; j<4; j++){
                T12i_eig(i, j)=pose_mat.at<float>(i, j);
            }
        }
    }
    
    int CheckInliers(std::vector<Eigen::Vector3d>& P1,
                                     std::vector<Eigen::Vector3d>& P2,
                                     const Eigen::Matrix4d& T12,
                                     const double& scale,
                                     std::vector<bool> &b_inliners)
    {
        int size_position = P1.size();
        int inlier = 0;
        if (std::isinf(scale)) {
            return inlier;
        }
        for (int i = 0; i < size_position; i++) {
            Eigen::Vector3d local_to_global;
            TransformPositionUseSim3(T12, scale, P2[i], local_to_global);
            double tmp = sqrt(pow(P1[i][0] - local_to_global[0], 2) +
                            pow(P1[i][1] - local_to_global[1], 2) +
                            pow(P1[i][2] - local_to_global[2], 2));
            if(tmp < 2.0){
                b_inliners[i] = true;
                inlier++;
            }
        }

        return inlier;
    }
    
    int RandomInt(int min, int max){
        int d = max - min + 1;
        return int(((double)rand()/((double)RAND_MAX + 1.0)) * d) + min;
    }
    
    bool ComputeSim3Ransac(std::vector<Eigen::Vector3d>& P1,
                                           std::vector<Eigen::Vector3d>& P2,
                                           Eigen::Matrix4d& T12,
                                           double& scale_12)
    { 
        CHECK_EQ(P1.size(), P2.size());
        if(P1.size()<10){
            std::cout<<"sim3 ransac too few points!!"<<std::endl;
            return false;
        }
        std::vector<size_t> all_indices;
        int match_size = P1.size();
        all_indices.reserve(match_size);
        for (int i = 0; i < match_size; i++) {
            all_indices.push_back(i);
        }

        float ransac_prob = 0.99;
        int ransac_min_inliers = 50;
        int ransac_max_iter = 1000;

        // Adjust Parameters according to number of correspondences
        float epsilon = (float)ransac_min_inliers / match_size;

        // Set RANSAC iterations according to probability, epsilon, and max iterations
        int n_iterations;

//         if (ransac_min_inliers == match_size)
//             n_iterations = 1;
//         else
//             n_iterations = ceil(log(1 - ransac_prob) / log(1 - pow(epsilon, 3)));

        //ransac_max_iter = max(1, min(n_iterations, ransac_max_iter));
        ransac_max_iter=1000;
        int count_iter = 0;
        int ninliers_final=0;
        while (count_iter < ransac_max_iter) {
            count_iter++;

            std::vector<size_t> available_indices;
            available_indices = all_indices;

            std::vector<Eigen::Vector3d> P3D1, P3D2;
            P3D1.reserve(3);
            P3D2.reserve(3);
            // Get min set of points
            for (short i = 0; i < 3; ++i) {
                int randi = RandomInt(0, available_indices.size() - 1);
                CHECK_GT(available_indices.size(), randi);
                int idx = available_indices[randi];
                CHECK_GT(P1.size(), idx);
                CHECK_GT(P2.size(), idx);
                P3D1.push_back(P1[idx]);
                P3D2.push_back(P2[idx]);

                available_indices[randi] = available_indices.back();
                available_indices.pop_back();
            }

            ComputeSim3(P3D1, P3D2, T12, scale_12);

            std::vector<bool> b_inliners = std::vector<bool>(match_size,false);
            int ninliers = CheckInliers(P1, P2, T12, scale_12, b_inliners);
            //std::cout<<"ransac ninliers: "<<ninliers<<std::endl; 
            if (ninliers > ransac_min_inliers) {
                std::vector<Eigen::Vector3d> P3D1_inlier, P3D2_inlier;
                // std::vector<cv::KeyPoint> local_kp_inlier;
                P3D1_inlier.reserve(match_size);
                P3D2_inlier.reserve(match_size);
                // local_kp.reserve(match_size);
                for (int i = 0; i < match_size; i++){
                    CHECK_GT(b_inliners.size(), i);
                    if (b_inliners[i]){
                        CHECK_GT(P1.size(), i);
                        CHECK_GT(P2.size(), i);
                        P3D1_inlier.push_back(P1[i]);
                        P3D2_inlier.push_back(P2[i]);
                        // local_kp_inlier.push_back(local_kp[i]);
                    }
                }

                ninliers_final = OptimizeSim3Align(P3D1_inlier,
                                                                        P3D2_inlier,
                                                                        T12,
                                                                        scale_12,
                                                                        2.0);
                
                // chamo::ComputeSim3(P3D1_inlier, P3D2_inlier, T12, scale_12);
                // int ninliers_final = CheckInliers(P3D1_inlier, P3D2_inlier, T12, scale_12, b_inliners);
                // if(ninliers_final > 0.5 * ninliers){
                if(ninliers_final > ransac_min_inliers){
                    return true;
                }
            }
        }
        if(ninliers_final<20){
            return false;
        }else{
            return true;
        }
    }
}
