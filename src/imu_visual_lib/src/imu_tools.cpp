#include <iostream>
#include <imu_tools.h>
#include "g2o/core/robust_kernel_impl.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/core/optimization_algorithm_gauss_newton.h"
#include "g2o/core/optimization_algorithm_with_hessian.h"
#include "g2o/solvers/linear_solver_dense.h"
#include "g2o/solvers/linear_solver_eigen.h"
#include "g2o/types/types_seven_dof_expmap.h"
#include "g2o/types/types_six_dof_expmap.h"

#include<Eigen/StdVector>
#include "configparam.h"
#include "g2otypes.h"


#include <memory>
#include "IMUConverter.h"

namespace chamo
{
    using namespace std;
    using namespace Eigen;

    cv::Mat SkewSymmetricMatrix(const cv::Mat &v)
    {
        return (cv::Mat_<float>(3,3) <<             0, -v.at<float>(2), v.at<float>(1),
                v.at<float>(2),               0,-v.at<float>(0),
                -v.at<float>(1),  v.at<float>(0),              0);
    }

    void CalAccBias(const vector<cv::Mat>& vTwc, const vector<IMUPreintegrator>& vImuPreInt,
                    double& sstar, cv::Mat& gwstar, cv::Mat Tbc, Eigen::Matrix3d& Rwi_refined, Eigen::Vector3d& bias_a){
        int N= vTwc.size();
        cv::Mat Rbc = Tbc.rowRange(0,3).colRange(0,3);
        cv::Mat pbc = Tbc.rowRange(0,3).col(3);
        cv::Mat Rcb = Rbc.t();
        cv::Mat pcb = -Rcb*pbc;
        cv::Mat gI = cv::Mat::zeros(3,1,CV_32F);
        gI.at<float>(2) = 1;
        // Normalized approx. gravity vecotr in world frame
        cv::Mat gwn = gwstar/cv::norm(gwstar);
        // Debug log
        //cout<<"gw normalized: "<<gwn<<endl;

        // vhat = (gI x gw) / |gI x gw|
        cv::Mat gIxgwn = gI.cross(gwn);
        double normgIxgwn = cv::norm(gIxgwn);
        cv::Mat vhat = gIxgwn/normgIxgwn;
        double theta = std::atan2(normgIxgwn,gI.dot(gwn));
        // Debug log
        //cout<<"vhat: "<<vhat<<", theta: "<<theta*180.0/M_PI<<endl;

        Eigen::Vector3d vhateig = Converter::toVector3d(vhat);
        Eigen::Matrix3d RWIeig = Sophus::SO3::exp(vhateig*theta).matrix();
        cv::Mat Rwi = Converter::toCvMat(RWIeig);
        cv::Mat GI = gI*9.8;//9.8012;
        // Solve C*x=D for x=[s,dthetaxy,ba] (1+2+3)x1 vector
        cv::Mat C = cv::Mat::zeros(3*(N-2),6,CV_32F);
        cv::Mat D = cv::Mat::zeros(3*(N-2),1,CV_32F);

        for(int i=0; i<N-2; i++)
        {
            if(vImuPreInt[i+1].getDeltaTime()==0){
                continue;
            }
            if(vImuPreInt[i+2].getDeltaTime()==0){
                continue;
            }
            // Delta time between frames
            double dt12 = vImuPreInt[i+1].getDeltaTime();
            double dt23 = vImuPreInt[i+2].getDeltaTime();
            // Pre-integrated measurements
            cv::Mat dp12 = Converter::toCvMat(vImuPreInt[i+1].getDeltaP());
            cv::Mat dv12 = Converter::toCvMat(vImuPreInt[i+1].getDeltaV());
            cv::Mat dp23 = Converter::toCvMat(vImuPreInt[i+2].getDeltaP());
            cv::Mat Jpba12 = Converter::toCvMat(vImuPreInt[i+1].getJPBiasa());
            cv::Mat Jvba12 = Converter::toCvMat(vImuPreInt[i+1].getJVBiasa());
            cv::Mat Jpba23 = Converter::toCvMat(vImuPreInt[i+2].getJPBiasa());
            // Pose of camera in world frame
            cv::Mat Twc1 = vTwc[i].clone();//pKF1->GetPoseInverse();
            cv::Mat Twc2 = vTwc[i+1].clone();//pKF2->GetPoseInverse();
            cv::Mat Twc3 = vTwc[i+2].clone();//pKF3->GetPoseInverse();
            // Position of camera center
            cv::Mat pc1 = Twc1.rowRange(0,3).col(3);
            cv::Mat pc2 = Twc2.rowRange(0,3).col(3);
            cv::Mat pc3 = Twc3.rowRange(0,3).col(3);
            // Rotation of camera, Rwc
            cv::Mat Rc1 = Twc1.rowRange(0,3).colRange(0,3);
            cv::Mat Rc2 = Twc2.rowRange(0,3).colRange(0,3);
            cv::Mat Rc3 = Twc3.rowRange(0,3).colRange(0,3);
            // Stack to C/D matrix
            // lambda*s + phi*dthetaxy + zeta*ba = psi
            cv::Mat lambda = (pc2-pc1)*dt23 + (pc2-pc3)*dt12;
            cv::Mat phi = - 0.5*(dt12*dt12*dt23 + dt12*dt23*dt23)*Rwi*SkewSymmetricMatrix(GI);  // note: this has a '-', different to paper
            cv::Mat zeta = Rc2*Rcb*Jpba23*dt12 + Rc1*Rcb*Jvba12*dt12*dt23 - Rc1*Rcb*Jpba12*dt23;
            cv::Mat psi = (Rc1-Rc2)*pcb*dt23 + Rc1*Rcb*dp12*dt23 - (Rc2-Rc3)*pcb*dt12
                         - Rc2*Rcb*dp23*dt12 - Rc1*Rcb*dv12*dt23*dt12 - 0.5*Rwi*GI*(dt12*dt12*dt23 + dt12*dt23*dt23); // note:  - paper
            lambda.copyTo(C.rowRange(3*i+0,3*i+3).col(0));
            phi.colRange(0,2).copyTo(C.rowRange(3*i+0,3*i+3).colRange(1,3)); //only the first 2 columns, third term in dtheta is zero, here compute dthetaxy 2x1.
            zeta.copyTo(C.rowRange(3*i+0,3*i+3).colRange(3,6));
            psi.copyTo(D.rowRange(3*i+0,3*i+3));

            // Debug log
            //cout<<"iter "<<i<<endl;
        }

        // Use svd to compute C*x=D, x=[s,dthetaxy,ba] 6x1 vector
        // C = u*w*vt, u*w*vt*x=D
        // Then x = vt'*winv*u'*D
        cv::Mat w2,u2,vt2;
        // Note w2 is 6x1 vector by SVDecomp()
        // C is changed in SVDecomp() with cv::SVD::MODIFY_A for speed
        cv::SVDecomp(C,w2,u2,vt2,cv::SVD::MODIFY_A);
        // Debug log
        //cout<<"u2:"<<endl<<u2<<endl;
        //cout<<"vt2:"<<endl<<vt2<<endl;
        //cout<<"w2:"<<endl<<w2<<endl;

        // Compute winv
        cv::Mat w2inv=cv::Mat::eye(6,6,CV_32F);
        for(int i=0;i<6;i++)
        {
            if(fabs(w2.at<float>(i))<1e-10)
            {
                w2.at<float>(i) += 1e-10;
                // Test log
                cerr<<"w2(i) < 1e-10, w="<<endl<<w2<<endl;
            }

            w2inv.at<float>(i,i) = 1./w2.at<float>(i);
        }
        // Then y = vt'*winv*u'*D
        cv::Mat y = vt2.t()*w2inv*u2.t()*D;

        double s_ = y.at<float>(0);
        sstar=s_;
        cv::Mat dthetaxy = y.rowRange(1,3);
        cv::Mat dbiasa_ = y.rowRange(3,6);
        
        //cv::Mat dbiasa_=cv::Mat::zeros(3,1, CV_64FC1);
        Vector3d dbiasa_eig = Converter::toVector3d(dbiasa_);
        bias_a=dbiasa_eig;
        // dtheta = [dx;dy;0]
        cv::Mat dtheta = cv::Mat::zeros(3,1,CV_32F);
        dthetaxy.copyTo(dtheta.rowRange(0,2));
        Eigen::Vector3d dthetaeig = Converter::toVector3d(dtheta);
        // Rwi_ = Rwi*exp(dtheta)
        Eigen::Matrix3d Rwieig_ = RWIeig*Sophus::SO3::exp(dthetaeig).matrix();
        Rwi_refined=Rwieig_;
    }

    void CalGravityAndScale(const vector<cv::Mat>& vTwc, const vector<IMUPreintegrator>& vImuPreInt, cv::Mat Tbc,
        double& sstar, cv::Mat& gwstar, double& scale_confi, double& grav_confi
    ){
        int N= vTwc.size();
        cv::Mat A = cv::Mat::zeros(3*(N-2),4,CV_32F);
        cv::Mat B = cv::Mat::zeros(3*(N-2),1,CV_32F);
        cv::Mat I3 = cv::Mat::eye(3,3,CV_32F);
        cv::Mat Rbc = Tbc.rowRange(0,3).colRange(0,3);
        cv::Mat pbc = Tbc.rowRange(0,3).col(3);
        cv::Mat Rcb = Rbc.t();
        cv::Mat pcb = -Rcb*pbc;

        for(int i=0; i<N-2; i++)
        {
            if(vImuPreInt[i+1].getDeltaTime()==0){
                continue;
            }
            if(vImuPreInt[i+2].getDeltaTime()==0){
                continue;
            }
            double dt12 = vImuPreInt[i+1].getDeltaTime();
            double dt23 = vImuPreInt[i+2].getDeltaTime();
            // Pre-integrated measurements
            cv::Mat dp12 = Converter::toCvMat(vImuPreInt[i+1].getDeltaP());
            cv::Mat dv12 = Converter::toCvMat(vImuPreInt[i+1].getDeltaV());
            cv::Mat dp23 = Converter::toCvMat(vImuPreInt[i+2].getDeltaP());

            // Pose of camera in world frame
            cv::Mat Twc1 = vTwc[i].clone();//pKF1->GetPoseInverse();
            cv::Mat Twc2 = vTwc[i+1].clone();//pKF2->GetPoseInverse();
            cv::Mat Twc3 = vTwc[i+2].clone();//pKF3->GetPoseInverse();
            // Position of camera center
            cv::Mat pc1 = Twc1.rowRange(0,3).col(3);
            cv::Mat pc2 = Twc2.rowRange(0,3).col(3);
            cv::Mat pc3 = Twc3.rowRange(0,3).col(3);
            // Rotation of camera, Rwc
            cv::Mat Rc1 = Twc1.rowRange(0,3).colRange(0,3);
            cv::Mat Rc2 = Twc2.rowRange(0,3).colRange(0,3);
            cv::Mat Rc3 = Twc3.rowRange(0,3).colRange(0,3);

            // Stack to A/B matrix
            // lambda*s + beta*g = gamma
            cv::Mat lambda = (pc2-pc1)*dt23 + (pc2-pc3)*dt12;
            cv::Mat beta = 0.5*I3*(dt12*dt12*dt23 + dt12*dt23*dt23);
            cv::Mat gamma = (Rc3-Rc2)*pcb*dt12 + (Rc1-Rc2)*pcb*dt23 + Rc1*Rcb*dp12*dt23 - Rc2*Rcb*dp23*dt12 - Rc1*Rcb*dv12*dt12*dt23;
            lambda.copyTo(A.rowRange(3*i+0,3*i+3).col(0));
            beta.copyTo(A.rowRange(3*i+0,3*i+3).colRange(1,4));
            gamma.copyTo(B.rowRange(3*i+0,3*i+3));
            // Tested the formulation in paper, -gamma. Then the scale and gravity vector is -xx

            // Debug log
            //cout<<"iter "<<i<<endl;
        }
        
        //Eigen::JacobiSVD<MatrixXd> svd(A);
        
        // Use svd to compute A*x=B, x=[s,gw] 4x1 vector
        // A = u*w*vt, u*w*vt*x=B
        // Then x = vt'*winv*u'*B
        cv::Mat w,u,vt;
        // Note w is 4x1 vector by SVDecomp()
        // A is changed in SVDecomp() with cv::SVD::MODIFY_A for speed
        cv::SVDecomp(A,w,u,vt,cv::SVD::MODIFY_A);
        //double cond = w.at<float>(0) / w.at<float>(3);
        //std::cout<<"cond: "<<cond<<std::endl;
        // Debug log
        //cout<<"u:"<<endl<<u<<endl;
        //cout<<"vt:"<<endl<<vt<<endl;
        //cout<<"w:"<<w.t()<<endl;

        // Compute winv
        cv::Mat winv=cv::Mat::eye(4,4,CV_32F);
        for(int i=0;i<4;i++)
        {
            if(fabs(w.at<float>(i))<1e-10)
            {
                w.at<float>(i) += 1e-10;
                // Test log
                cerr<<"w(i) < 1e-10, w="<<endl<<w<<endl;
            }

            winv.at<float>(i,i) = 1./w.at<float>(i);
        }
        // Then x = vt'*winv*u'*B
        cv::Mat x = vt.t()*winv*u.t()*B;
        cv::Mat x1=x.clone();
        x1.at<float>(0)=x.at<float>(0)*0.7;
        double err = cv::norm(A*x-A*x1)/B.rows;
        scale_confi=err;
        x1=x.clone();
        x1.at<float>(1)=x.at<float>(1)*0.7;
        x1.at<float>(2)=x.at<float>(2)*0.7;
        x1.at<float>(3)=x.at<float>(3)*0.7;
        grav_confi=err = cv::norm(A*x-A*x1)/B.rows;
        
        //std::cout<<B.rows<<std::endl;

        // x=[s,gw] 4x1 vector
        sstar = x.at<float>(0);    // scale should be positive
        gwstar = x.rowRange(1,4);   // gravity should be about ~9.8
    }

    

    Eigen::Vector3d OptimizeInitialGyroBias(const vector<cv::Mat>& vTwc, const vector<IMUPreintegrator>& vImuPreInt, Matrix4d Tbc)
    {
        int N = vTwc.size(); if(vTwc.size()!=vImuPreInt.size()) cerr<<"vTwc.size()!=vImuPreInt.size()"<<endl;
        
        Matrix3d Rcb = Tbc.topLeftCorner(3,3).transpose();
        
        g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(
            new g2o::BlockSolverX(
                new g2o::LinearSolverEigen<g2o::BlockSolverX::PoseMatrixType>()));

        g2o::SparseOptimizer optimizer;
        optimizer.setAlgorithm(solver);

        // Add vertex of gyro bias, to optimizer graph
        g2o::VertexGyrBias * vBiasg = new g2o::VertexGyrBias();
        vBiasg->setEstimate(Eigen::Vector3d::Zero());
        vBiasg->setId(0);
        optimizer.addVertex(vBiasg);

        // Add unary edges for gyro bias vertex
        //for(std::vector<KeyFrame*>::const_iterator lit=vpKFs.begin(), lend=vpKFs.end(); lit!=lend; lit++)
        for(int i=0; i<N; i++)
        {
            // Ignore the first KF
            if(i==0)
                continue;

            const cv::Mat& Twi = vTwc[i-1];    // pose of previous KF
            Matrix3d Rwci = Converter::toMatrix3d(Twi.rowRange(0,3).colRange(0,3));
            //Matrix3d Rwci = Twi.rotation_matrix();
            const cv::Mat& Twj = vTwc[i];        // pose of this KF
            Matrix3d Rwcj = Converter::toMatrix3d(Twj.rowRange(0,3).colRange(0,3));
            //Matrix3d Rwcj =Twj.rotation_matrix();

            const IMUPreintegrator& imupreint = vImuPreInt[i];
            if(imupreint.getDeltaTime()==0){
                continue;
            }

            g2o::EdgeGyrBias * eBiasg = new g2o::EdgeGyrBias();
            eBiasg->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(0)));
            // measurement is not used in EdgeGyrBias
            eBiasg->dRbij = imupreint.getDeltaR();
            eBiasg->J_dR_bg = imupreint.getJRBiasg();
            eBiasg->Rwbi = Rwci*Rcb;
            eBiasg->Rwbj = Rwcj*Rcb;
            //eBiasg->setInformation(Eigen::Matrix3d::Identity());
            eBiasg->setInformation(imupreint.getCovPVPhi().bottomRightCorner(3,3).inverse());
            optimizer.addEdge(eBiasg);
        }

        // It's actualy a linear estimator, so 1 iteration is enough.
        //optimizer.setVerbose(true);
        optimizer.initializeOptimization();
        optimizer.optimize(1);

        g2o::VertexGyrBias * vBgEst = static_cast<g2o::VertexGyrBias*>(optimizer.vertex(0));

        return vBgEst->estimate();
    }
    
    Eigen::Matrix3d calRotMFromGravity(Eigen::Vector3d gravity){
        Eigen::Vector3d gI = Eigen::Vector3d::Zero();
        gI(2) = 1;
        Eigen::Vector3d gwn = gravity/gravity.norm();
        Eigen::Vector3d gIxgwn = gI.cross(gwn);
        double normgIxgwn = gIxgwn.norm();
        Eigen::Vector3d vhat = gIxgwn/normgIxgwn;
        double theta = std::atan2(normgIxgwn,gI.dot(gwn));
        Eigen::Matrix3d Rwi_ = Sophus::SO3::exp(vhat*theta).matrix();
        return Rwi_;
    }
}
