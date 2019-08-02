// void GlobalBundleAdjustmentNavStatePRV(std::vector<IMUPreintegrator>& preints, std::vector<NavState>& states, Matrix4d Tbc,
//                                            std::vector<std::vector<MP_INFO>> mp_infos, float fx, float fy, float cx, float cy,
//                                            std::vector<Eigen::Vector3d>& mp_posis,
//                                            const cv::Mat& gw, int nIterations)
//     {
//         int nlevels=8;
//         float scaleFactor=1.2;
//         std::vector<float> mvScaleFactor;
//         std::vector<float> mvInvScaleFactor;
//         std::vector<float> mvLevelSigma2;
//         std::vector<float> mvInvLevelSigma2;
//         mvScaleFactor.resize(nlevels);
//         mvLevelSigma2.resize(nlevels);
//         mvScaleFactor[0]=1.0f;
//         mvLevelSigma2[0]=1.0f;
//         for(int i=1; i<nlevels; i++)
//         {
//             mvScaleFactor[i]=mvScaleFactor[i-1]*scaleFactor;
//             mvLevelSigma2[i]=mvScaleFactor[i]*mvScaleFactor[i];
//         }
// 
//         mvInvScaleFactor.resize(nlevels);
//         mvInvLevelSigma2.resize(nlevels);
//         for(int i=0; i<nlevels; i++)
//         {
//             mvInvScaleFactor[i]=1.0f/mvScaleFactor[i];
//             mvInvLevelSigma2[i]=1.0f/mvLevelSigma2[i];
//         }
//         
//         Matrix3d Rbc = Tbc.topLeftCorner(3,3);
//         Vector3d Pbc = Tbc.topRightCorner(3,1);
//         // Gravity vector in world frame
//         Vector3d GravityVec = Converter::toVector3d(gw);
// 
//         vector<bool> vbNotIncludedMP;
//         vbNotIncludedMP.resize(mp_posis.size());
// 
//         g2o::SparseOptimizer optimizer;
//         g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(
//             new g2o::BlockSolverX(
//                 new g2o::LinearSolverEigen<g2o::BlockSolverX::PoseMatrixType>()));
//         optimizer.setAlgorithm(solver);
// 
//         long unsigned int maxKFid = 0;
// 
//         // Set KeyFrame vertices
//         for(size_t i=0; i<states.size(); i++)
//         {
//             //std::cout<<states[i].Get_R()<<std::endl;
//             g2o::VertexNavStatePR * vNSPR = new g2o::VertexNavStatePR();
//             vNSPR->setEstimate(states[i]);
//             vNSPR->setId(i*3);
//             vNSPR->setFixed(i==0);
//             optimizer.addVertex(vNSPR);
//             // V
//             g2o::VertexNavStateV * vNSV = new g2o::VertexNavStateV();
//             vNSV->setEstimate(states[i]);
//             vNSV->setId(i*3+1);
//             vNSV->setFixed(false);
//             optimizer.addVertex(vNSV);
//             // Bias
//             g2o::VertexNavStateBias * vNSBias = new g2o::VertexNavStateBias();
//             vNSBias->setEstimate(states[i]);
//             vNSBias->setId(i*3+2);
//             vNSBias->setFixed(i==0);
//             optimizer.addVertex(vNSBias);
// 
//             if(i*3+2>maxKFid)
//                 maxKFid=i*3+2;
//         }
// 
//         // Add NavState PRV/Bias edges
//         const float thHuberNavStatePRV = sqrt(100*21.666);
//         const float thHuberNavStateBias = sqrt(100*16.812);
//         // Inverse covariance of bias random walk
// 
//         Matrix<double,6,6> InvCovBgaRW = Matrix<double,6,6>::Identity();
//         InvCovBgaRW.topLeftCorner(3,3) = Matrix3d::Identity()/IMUData::getGyrBiasRW2();       // Gyroscope bias random walk, covariance INVERSE
//         InvCovBgaRW.bottomRightCorner(3,3) = Matrix3d::Identity()/IMUData::getAccBiasRW2();   // Accelerometer bias random walk, covariance INVERSE
// 
//         std::vector<g2o::EdgeNavStatePRV*> prvEdges;
//         for(size_t i=1; i<preints.size(); i++)
//         {
//             if(preints[i].getDeltaTime()==0){
//                 continue;
//             }
//             // PVR edge
//             {
//                 // PR0, PR1, V0, V1, B0
//                 g2o::EdgeNavStatePRV * epvr = new g2o::EdgeNavStatePRV();
//                 epvr->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(3*(i-1))));
//                 epvr->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(3*i)));
//                 epvr->setVertex(2, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(3*(i-1)+1)));
//                 epvr->setVertex(3, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(3*i+1)));
//                 epvr->setVertex(4, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(3*(i-1)+2)));
//                 epvr->setMeasurement(preints[i]);
// 
//                 Matrix9d CovPRV = preints[i].getCovPVPhi();
//                 CovPRV.col(3).swap(CovPRV.col(6));
//                 CovPRV.col(4).swap(CovPRV.col(7));
//                 CovPRV.col(5).swap(CovPRV.col(8));
//                 CovPRV.row(3).swap(CovPRV.row(6));
//                 CovPRV.row(4).swap(CovPRV.row(7));
//                 CovPRV.row(5).swap(CovPRV.row(8));
//                 epvr->setInformation(CovPRV.inverse());
//                 //epvr->setInformation(Matrix9d::Identity());
//                 epvr->SetParams(GravityVec);
// 
//                 g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
//                 epvr->setRobustKernel(rk);
//                 rk->setDelta(thHuberNavStatePRV);
//                 prvEdges.push_back(epvr);
//                 optimizer.addEdge(epvr);
//                 epvr->computeError();
//             }
//             // Bias edge
//             {
//                 g2o::EdgeNavStateBias * ebias = new g2o::EdgeNavStateBias();
//                 ebias->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(3*(i-1)+2)));
//                 ebias->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(3*i+2)));
//                 ebias->setMeasurement(preints[i]);
// 
//                 ebias->setInformation(InvCovBgaRW/preints[i].getDeltaTime());
// 
//                 g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
//                 ebias->setRobustKernel(rk);
//                 rk->setDelta(thHuberNavStateBias);
// 
//                 optimizer.addEdge(ebias);
//             }
// 
//         }
// 
//         const float thHuber2D = sqrt(5.99);
//         // Set MapPoint vertices
//         std::vector<g2o::EdgeNavStatePRPointXYZ*> project_edges;
//         for(size_t i=0; i<mp_infos.size(); i++)
//         {
//             //std::cout<<"i: "<<i<<std::endl;
//             g2o::VertexSBAPointXYZ* vPoint = new g2o::VertexSBAPointXYZ();
//             vPoint->setEstimate(mp_posis[i]);
//             const int id = i+maxKFid+1;
//             vPoint->setId(id);
//             vPoint->setMarginalized(true);
//             optimizer.addVertex(vPoint);
//             int nEdges = 0;
//             //SET EDGES
//             
//             for(int j=0; j<mp_infos[i].size(); j++)
//             {
//                 
//                 nEdges++;
//                 Eigen::Matrix<double,2,1> obs;
//                 obs << mp_infos[i][j].u, mp_infos[i][j].v;
// 
//                 g2o::EdgeNavStatePRPointXYZ* e = new g2o::EdgeNavStatePRPointXYZ();
//                 e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id)));
//                 e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(3*mp_infos[i][j].frame_id)));
//                 e->setMeasurement(obs);
//                 //std::cout<<"j: "<<mp_infos[i][j].octove<<std::endl;
//                 const float &invSigma2 = mvInvLevelSigma2[mp_infos[i][j].octove];
//                 
//                 e->setInformation(Eigen::Matrix2d::Identity()*invSigma2);
//                 //e->setInformation(Eigen::Matrix2d::Identity());
// 
//                 g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
//                 e->setRobustKernel(rk);
//                 rk->setDelta(thHuber2D);
// 
//                 e->SetParams(fx,fy,cx,cy,Rbc,Pbc);
//                 project_edges.push_back(e);
// 
//                 optimizer.addEdge(e);
//                 
//             }
// 
//             if(nEdges==0)
//             {
//                 optimizer.removeVertex(vPoint);
//                 vbNotIncludedMP[i]=true;
//             }
//             else
//             {
//                 vbNotIncludedMP[i]=false;
//             }
//         }
//         
//         float totall_pvr_error=0;
//         for(int i=0; i<prvEdges.size(); i++){
//             prvEdges[i]->computeError();
//             totall_pvr_error = totall_pvr_error+sqrt(prvEdges[i]->chi2())/prvEdges.size();
//         }
//         std::cout<<"totall_pvr_error before: "<<totall_pvr_error<<std::endl;
//         
//         float totall_proj_error=0;
//         for(int i=0; i<project_edges.size(); i++){
//             project_edges[i]->computeError();
//             totall_proj_error = totall_proj_error+sqrt(project_edges[i]->chi2())/project_edges.size();
//         }
//         std::cout<<"totall_proj_error before: "<<totall_proj_error<<std::endl;
//         
//         // Optimize!
//         optimizer.initializeOptimization();
//         optimizer.optimize(100);
//         totall_pvr_error=0;
//         for(int i=0; i<prvEdges.size(); i++){
//             prvEdges[i]->computeError();
//             totall_pvr_error = totall_pvr_error+sqrt(prvEdges[i]->chi2())/prvEdges.size();
//         }
//         std::cout<<"totall_pvr_error after: "<<totall_pvr_error<<std::endl;
//         
//         
//         totall_proj_error=0;
//         for(int i=0; i<project_edges.size(); i++){
//             project_edges[i]->computeError();
//             totall_proj_error = totall_proj_error+sqrt(project_edges[i]->chi2())/project_edges.size();
//         }
//         std::cout<<"totall_proj_error after: "<<totall_proj_error<<std::endl;
//         // Recover optimized data
//         //Keyframes
//         for(size_t i=0; i<states.size(); i++)
//         {
//             g2o::VertexNavStatePR* vNSPR = static_cast<g2o::VertexNavStatePR*>(optimizer.vertex(3*i));
//             g2o::VertexNavStateV* vNSV = static_cast<g2o::VertexNavStateV*>(optimizer.vertex(3*i+1));
//             g2o::VertexNavStateBias* vNSBias = static_cast<g2o::VertexNavStateBias*>(optimizer.vertex(3*i+2));
//             const NavState& nspr = vNSPR->estimate();
//             const NavState& nsv = vNSV->estimate();
//             const NavState& nsbias = vNSBias->estimate();
//             NavState& ns_recov = states[i];
//             ns_recov.Set_Pos(nspr.Get_P());
//             ns_recov.Set_Rot(nspr.Get_R());
//             ns_recov.Set_Vel(nsv.Get_V());
//             ns_recov.Set_BiasGyr(nsbias.Get_dBias_Gyr()+ nsbias.Get_dBias_Gyr());
//             ns_recov.Set_BiasAcc(nsbias.Get_dBias_Acc()+ nsbias.Get_dBias_Acc());
//         }
// 
//         //Points
//         for(size_t i=0; i<mp_posis.size(); i++)
//         {
//             if(vbNotIncludedMP[i])
//                 continue;
//             
//             g2o::VertexSBAPointXYZ* vPoint = static_cast<g2o::VertexSBAPointXYZ*>(optimizer.vertex(i+maxKFid+1));
//             mp_posis[i]=vPoint->estimate();
//         }
//     }
