#include "g2otypes.h"
#include <iostream>
namespace g2o
{

using namespace chamo;

void EdgePRIDP::computeError()
{
    const Vector3d Pi = computePc();

    // err = obs - pi(Px)
    _error = _measurement - cam_project(Pi);
}

Vector3d EdgePRIDP::computePc()
{
    const VertexIDP* vIDP = static_cast<const VertexIDP*>(_vertices[0]);
    const VertexNavStatePR* vPR0 = static_cast<const VertexNavStatePR*>(_vertices[1]);
    const VertexNavStatePR* vPRi = static_cast<const VertexNavStatePR*>(_vertices[2]);
    const VertexNavStatePR* vPRcb = static_cast<const VertexNavStatePR*>(_vertices[3]);

    //
    const Matrix3d R0 = vPR0->estimate().Get_RotMatrix();
    const Vector3d t0 = vPR0->estimate().Get_P();
    const Matrix3d Ri = vPRi->estimate().Get_RotMatrix();
    const Vector3d ti = vPRi->estimate().Get_P();
    const Matrix3d Rcb = vPRcb->estimate().Get_RotMatrix();
    const Vector3d tcb = vPRcb->estimate().Get_P();

    // point inverse depth in reference KF
    double rho = vIDP->estimate();
    if(rho<1e-6)
    {
        std::cerr<<"1. rho = "<<rho<<", rho<1e-6, shoudn't"<<std::endl;
        rho = 1e-6;
        setLevel(1);
    }
    // point coordinate in reference KF, body
    Vector3d P0;
    P0 << refnormxy[0], refnormxy[1], 1;
    double d=1.0/rho;   // depth
    P0 *= d;

    // Pi = Rcb*Ri^T*R0*Rcb^T* p0 + ( tcb - Rcb*Ri^T*R0*Rcb^T *tcb + Rcb*Ri^T*(t0-ti) )
    const Matrix3d Rcic0 = Rcb*Ri.transpose()*R0*Rcb.transpose();
    const Vector3d Pi = Rcic0*P0 + tcb - Rcic0*tcb + Rcb*Ri.transpose()*(t0-ti);
    return Pi;
}

void EdgeNavStatePRV::computeError()
{
    //
    
    const VertexNavStatePR* vPRi = static_cast<const VertexNavStatePR*>(_vertices[0]);
    const VertexNavStatePR* vPRj = static_cast<const VertexNavStatePR*>(_vertices[1]);
    const VertexNavStateV* vVi = static_cast<const VertexNavStateV*>(_vertices[2]);
    const VertexNavStateV* vVj = static_cast<const VertexNavStateV*>(_vertices[3]);
    const VertexNavStateBias* vBiasi = static_cast<const VertexNavStateBias*>(_vertices[4]);

    // terms need to computer error in vertex i, except for bias error
    const NavState& NSPRi = vPRi->estimate();
    const Vector3d Pi = NSPRi.Get_P();
    const Sophus::SO3 Ri = NSPRi.Get_R();

    const NavState& NSVi = vVi->estimate();
    const Vector3d Vi = NSVi.Get_V();
    // Bias from the bias vertex
    const NavState& NSBiasi = vBiasi->estimate();
    const Vector3d dBgi = NSBiasi.Get_dBias_Gyr();
    const Vector3d dBai = NSBiasi.Get_dBias_Acc();

    // terms need to computer error in vertex j, except for bias error
    const NavState& NSPRj = vPRj->estimate();
    const Vector3d Pj = NSPRj.Get_P();
    const Sophus::SO3 Rj = NSPRj.Get_R();

    const NavState& NSVj = vVj->estimate();
    const Vector3d Vj = NSVj.Get_V();

    // IMU Preintegration measurement
    const IMUPreintegrator& M = _measurement;
    const double dTij = M.getDeltaTime();   // Delta Time
    const double dT2 = dTij*dTij;
    const Vector3d dPij = M.getDeltaP();    // Delta Position pre-integration measurement
    const Vector3d dVij = M.getDeltaV();    // Delta Velocity pre-integration measurement
    const Sophus::SO3 dRij = Sophus::SO3(M.getDeltaR());  // Delta Rotation pre-integration measurement

    // tmp variable, transpose of Ri
    const Sophus::SO3 RiT = Ri.inverse();
    // residual error of Delta Position measurement
    const Vector3d rPij = RiT*(Pj - Pi - Vi*dTij - 0.5*GravityVec*dT2)
                          - (dPij + M.getJPBiasg()*dBgi + M.getJPBiasa()*dBai);   // this line includes correction term of bias change.
    // residual error of Delta Velocity measurement
    const Vector3d rVij = RiT*(Vj - Vi - GravityVec*dTij)
                          - (dVij + M.getJVBiasg()*dBgi + M.getJVBiasa()*dBai);   //this line includes correction term of bias change
    // residual error of Delta Rotation measurement
    const Sophus::SO3 dR_dbg = Sophus::SO3::exp(M.getJRBiasg()*dBgi);
    const Sophus::SO3 rRij = (dRij * dR_dbg).inverse() * RiT * Rj;
    const Vector3d rPhiij = rRij.log();


    Vector9d err;  // typedef Matrix<double, D, 1> ErrorVector; ErrorVector _error; D=9
    err.setZero();

    // 9-Dim error vector order:
    // position-velocity-rotation
    // rPij - rPhiij - rVij
    err.segment<3>(0) = rPij;       // position error
    err.segment<3>(3) = rPhiij;     // rotation phi error
    err.segment<3>(6) = rVij;       // velocity error
    //std::cout<<rPij.norm()<<":"<<rPhiij.norm()<<":"<<rVij.norm()<<std::endl;
    _error = err;
}

void EdgeNavStatePriorPRVBias::computeError()
{
    const VertexNavStatePR* vNSPR = static_cast<const VertexNavStatePR*>(_vertices[0]);
    const VertexNavStateV* vNSV = static_cast<const VertexNavStateV*>(_vertices[1]);
    const VertexNavStateBias* vNSBias = static_cast<const VertexNavStateBias*>(_vertices[2]);
    const NavState& nsPRest = vNSPR->estimate();
    const NavState& nsVest = vNSV->estimate();
    const NavState& nsBiasest = vNSBias->estimate();
    const NavState& nsprior = _measurement;

    // P V R bg+dbg ba+dba
    Vector15d err = Vector15d::Zero();

    // PVR terms
    // eP = P_prior - P_est
    err.segment<3>(0) = nsprior.Get_P() - nsPRest.Get_P();
    // eR = log(R_prior^-1 * R_est)
    err.segment<3>(3) = (nsprior.Get_R().inverse() * nsPRest.Get_R()).log();
    // eV = V_prior - V_est
    err.segment<3>(6) = nsprior.Get_V() - nsVest.Get_V();

    // Bias terms
    // eB = Bias_prior - Bias_est
    // err_bg = (bg_prior+dbg_prior) - (bg+dbg)
    err.segment<3>(9) = (nsprior.Get_BiasGyr() + nsprior.Get_dBias_Gyr()) - (nsBiasest.Get_BiasGyr() + nsBiasest.Get_dBias_Gyr());
    // err_ba = (ba_prior+dba_prior) - (ba+dba)
    err.segment<3>(12) = (nsprior.Get_BiasAcc() + nsprior.Get_dBias_Acc()) - (nsBiasest.Get_BiasAcc() + nsBiasest.Get_dBias_Acc());


    _error = err;

}

void EdgeNavStatePVR::computeError()
{
    //
    const VertexNavStatePVR* vPVRi = static_cast<const VertexNavStatePVR*>(_vertices[0]);
    const VertexNavStatePVR* vPVRj = static_cast<const VertexNavStatePVR*>(_vertices[1]);
    const VertexNavStateBias* vBiasi = static_cast<const VertexNavStateBias*>(_vertices[2]);

    // terms need to computer error in vertex i, except for bias error
    const NavState& NSPVRi = vPVRi->estimate();
    Vector3d Pi = NSPVRi.Get_P();
    Vector3d Vi = NSPVRi.Get_V();
    Sophus::SO3 Ri = NSPVRi.Get_R();
    // Bias from the bias vertex
    const NavState& NSBiasi = vBiasi->estimate();
    Vector3d dBgi = NSBiasi.Get_dBias_Gyr();
    Vector3d dBai = NSBiasi.Get_dBias_Acc();

    // terms need to computer error in vertex j, except for bias error
    const NavState& NSPVRj = vPVRj->estimate();
    Vector3d Pj = NSPVRj.Get_P();
    Vector3d Vj = NSPVRj.Get_V();
    Sophus::SO3 Rj = NSPVRj.Get_R();

    // IMU Preintegration measurement
    const IMUPreintegrator& M = _measurement;
    double dTij = M.getDeltaTime();   // Delta Time
    double dT2 = dTij*dTij;
    Vector3d dPij = M.getDeltaP();    // Delta Position pre-integration measurement
    Vector3d dVij = M.getDeltaV();    // Delta Velocity pre-integration measurement
    Sophus::SO3 dRij = Sophus::SO3(M.getDeltaR());  // Delta Rotation pre-integration measurement

    // tmp variable, transpose of Ri
    Sophus::SO3 RiT = Ri.inverse();
    // residual error of Delta Position measurement
    Vector3d rPij = RiT*(Pj - Pi - Vi*dTij - 0.5*GravityVec*dT2)
                  - (dPij + M.getJPBiasg()*dBgi + M.getJPBiasa()*dBai);   // this line includes correction term of bias change.
    // residual error of Delta Velocity measurement
    Vector3d rVij = RiT*(Vj - Vi - GravityVec*dTij)
                  - (dVij + M.getJVBiasg()*dBgi + M.getJVBiasa()*dBai);   //this line includes correction term of bias change
    // residual error of Delta Rotation measurement
    Sophus::SO3 dR_dbg = Sophus::SO3::exp(M.getJRBiasg()*dBgi);
    Sophus::SO3 rRij = (dRij * dR_dbg).inverse() * RiT * Rj;
    Vector3d rPhiij = rRij.log();


    Vector9d err;  // typedef Matrix<double, D, 1> ErrorVector; ErrorVector _error; D=9
    err.setZero();

    // 9-Dim error vector order:
    // position-velocity-rotation
    // rPij - rVij - rPhiij
    err.segment<3>(0) = rPij;       // position error
    err.segment<3>(3) = rVij;       // velocity error
    err.segment<3>(6) = rPhiij;     // rotation phi error

    _error = err;

    //Test log
    if( (NSPVRi.Get_BiasGyr()-NSBiasi.Get_BiasGyr()).norm()>1e-6 || (NSPVRi.Get_BiasAcc()-NSBiasi.Get_BiasAcc()).norm()>1e-6 )
    {
        std::cerr<<"id pvri/pvrj/biasi: "<<vPVRi->id()<<"/"<<vPVRj->id()<<"/"<<vBiasi->id()<<std::endl;
        std::cerr<<"bias gyr not equal for PVR/Bias vertex"<<std::endl<<NSPVRi.Get_BiasGyr().transpose()<<" / "<<NSBiasi.Get_BiasGyr().transpose()<<std::endl;
        std::cerr<<"bias acc not equal for PVR/Bias vertex"<<std::endl<<NSPVRi.Get_BiasAcc().transpose()<<" / "<<NSBiasi.Get_BiasAcc().transpose()<<std::endl;
    }

}

void EdgeNavStateBias::computeError()
{
    //
    const VertexNavStateBias* vBiasi = static_cast<const VertexNavStateBias*>(_vertices[0]);
    const VertexNavStateBias* vBiasj = static_cast<const VertexNavStateBias*>(_vertices[1]);

    const NavState& NSi = vBiasi->estimate();
    const NavState& NSj = vBiasj->estimate();

    // residual error of Gyroscope's bias, Forster 15'RSS
    Vector3d rBiasG = (NSj.Get_BiasGyr() + NSj.Get_dBias_Gyr())
                    - (NSi.Get_BiasGyr() + NSi.Get_dBias_Gyr());

    // residual error of Accelerometer's bias, Forster 15'RSS
    Vector3d rBiasA = (NSj.Get_BiasAcc() + NSj.Get_dBias_Acc())
                    - (NSi.Get_BiasAcc() + NSi.Get_dBias_Acc());

    Vector6d err;  // typedef Matrix<double, D, 1> ErrorVector; ErrorVector _error; D=6
    err.setZero();
    // 6-Dim error vector order:
    // deltabiasGyr_i-deltabiasAcc_i
    // rBiasGi - rBiasAi
    err.segment<3>(0) = rBiasG;     // bias gyro error
    err.segment<3>(3) = rBiasA;    // bias acc error

    _error = err;
}

void EdgeNavStatePriorPVRBias::computeError()
{
    const VertexNavStatePVR* vNSPVR = static_cast<const VertexNavStatePVR*>(_vertices[0]);
    const VertexNavStateBias* vNSBias = static_cast<const VertexNavStateBias*>(_vertices[1]);
    const NavState& nsPVRest = vNSPVR->estimate();
    const NavState& nsBiasest = vNSBias->estimate();
    const NavState& nsprior = _measurement;

    // P V R bg+dbg ba+dba
    Vector15d err = Vector15d::Zero();

    // PVR terms
    // eP = P_prior - P_est
    err.segment<3>(0) = nsprior.Get_P() - nsPVRest.Get_P();
    // eV = V_prior - V_est
    err.segment<3>(3) = nsprior.Get_V() - nsPVRest.Get_V();
    // eR = log(R_prior^-1 * R_est)
    err.segment<3>(6) = (nsprior.Get_R().inverse() * nsPVRest.Get_R()).log();

    // Bias terms
    // eB = Bias_prior - Bias_est
    // err_bg = (bg_prior+dbg_prior) - (bg+dbg)
    err.segment<3>(9) = (nsprior.Get_BiasGyr() + nsprior.Get_dBias_Gyr()) - (nsBiasest.Get_BiasGyr() + nsBiasest.Get_dBias_Gyr());
    // err_ba = (ba_prior+dba_prior) - (ba+dba)
    err.segment<3>(12) = (nsprior.Get_BiasAcc() + nsprior.Get_dBias_Acc()) - (nsBiasest.Get_BiasAcc() + nsBiasest.Get_dBias_Acc());

    _error = err;

    //Debug log
    //std::cout<<"prior edge error: "<<std::endl<<_error.transpose()<<std::endl;

    //Test log
    if( (nsPVRest.Get_BiasGyr()-nsBiasest.Get_BiasGyr()).norm()>1e-6 || (nsPVRest.Get_BiasAcc()-nsBiasest.Get_BiasAcc()).norm()>1e-6 )
    {
        std::cerr<<"bias gyr not equal for PVR/Bias vertex in EdgeNavStatePriorPVRBias"<<std::endl<<nsPVRest.Get_BiasGyr().transpose()<<" / "<<nsBiasest.Get_BiasGyr().transpose()<<std::endl;
        std::cerr<<"bias acc not equal for PVR/Bias vertex in EdgeNavStatePriorPVRBias"<<std::endl<<nsPVRest.Get_BiasAcc().transpose()<<" / "<<nsBiasest.Get_BiasAcc().transpose()<<std::endl;
    }
}

//--------------------------------------

/**
 * @brief EdgeNavStatePrior::EdgeNavStatePrior
 */
void EdgeNavStatePrior::computeError()
{
    // Estimated NavState
    const VertexNavState* v = static_cast<const VertexNavState*>(_vertices[0]);
    const NavState& nsest = v->estimate();
    // Measurement: NavState_prior
    const NavState& nsprior = _measurement;

    // P V R bg+dbg ba+dba
    Vector15d err = Vector15d::Zero();

//    // err_P = P - P_prior
//    err.segment<3>(0) = nsest.Get_P() - nsprior.Get_P();
//    // err_V = V - V_prior
//    err.segment<3>(3) = nsest.Get_V() - nsprior.Get_V();
//    // err_R = log (R * R_prior^-1)
//    err.segment<3>(6) = (nsest.Get_R() * nsprior.Get_R().inverse()).log();
//    // err_bg = (bg+dbg) - (bg_prior+dbg_prior)
//    err.segment<3>(9) = (nsest.Get_BiasGyr() + nsest.Get_dBias_Gyr()) - (nsprior.Get_BiasGyr() + nsprior.Get_dBias_Gyr());
//    // err_ba = (ba+dba) - (ba_prior+dba_prior)
//    err.segment<3>(12) = (nsest.Get_BiasAcc() + nsest.Get_dBias_Acc()) - (nsprior.Get_BiasAcc() + nsprior.Get_dBias_Acc());

    // err_P = P - P_prior
    err.segment<3>(0) = nsprior.Get_P() - nsest.Get_P();
    // err_V = V - V_prior
    err.segment<3>(3) = nsprior.Get_V() - nsest.Get_V();
    // err_R = log (R * R_prior^-1)
    err.segment<3>(6) = (nsprior.Get_R().inverse() * nsest.Get_R()).log();
    // err_bg = (bg+dbg) - (bg_prior+dbg_prior)
    err.segment<3>(9) = (nsprior.Get_BiasGyr() + nsprior.Get_dBias_Gyr()) - (nsest.Get_BiasGyr() + nsest.Get_dBias_Gyr());
    // err_ba = (ba+dba) - (ba_prior+dba_prior)
    err.segment<3>(12) = (nsprior.Get_BiasAcc() + nsprior.Get_dBias_Acc()) - (nsest.Get_BiasAcc() + nsest.Get_dBias_Acc());

    _error = err;

    //Debug log
    //std::cout<<"prior edge error: "<<std::endl<<_error.transpose()<<std::endl;
}

/**
 * @brief VertexNavState::VertexNavState
 */
VertexNavState::VertexNavState() : BaseVertex<15, NavState>()
{
}
// Todo
bool VertexNavState::read(std::istream& is) {return true;}
bool VertexNavState::write(std::ostream& os) const {return true;}

void VertexNavState::oplusImpl(const double* update_)
{
    // 1.
    // order in 'update_'
    // dP, dV, dPhi, dBiasGyr, dBiasAcc

    // 2.
    // the same as Forster 15'RSS
    // pi = pi + dpi,    pj = pj + dpj
    // vi = vi + dvi,       vj = vj + dvj
    // Ri = Ri*Exp(dphi_i), Rj = Rj*Exp(dphi_j)
    //      Note: the optimized bias term is the 'delta bias'
    // delta_biasg_i = delta_biasg_i + dbgi,    delta_biasg_j = delta_biasg_j + dbgj
    // delta_biasa_i = delta_biasa_i + dbai,    delta_biasa_j = delta_biasa_j + dbaj

    Eigen::Map<const Vector15d> update(update_);
    _estimate.IncSmall(update);

    //std::cout<<"id "<<id()<<" ns update: "<<update.transpose()<<std::endl;
}

/**
 * @brief EdgeNavState::EdgeNavState
 */
EdgeNavState::EdgeNavState() : BaseBinaryEdge<15, IMUPreintegrator, VertexNavState, VertexNavState>()
{
}
// Todo
bool EdgeNavState::read(std::istream& is) {return true;}
bool EdgeNavState::write(std::ostream& os) const {return true;}

/**
 * @brief EdgeNavState::computeError
 * In g2o, computeError() is called in computeActiveErrors(), before buildSystem()
 */
void EdgeNavState::computeError()
{
    const VertexNavState* vi = static_cast<const VertexNavState*>(_vertices[0]);
    const VertexNavState* vj = static_cast<const VertexNavState*>(_vertices[1]);

    // terms need to computer error in vertex i, except for bias error
    const NavState& NSi = vi->estimate();
    Vector3d Pi = NSi.Get_P();
    Vector3d Vi = NSi.Get_V();
    //Matrix3d Ri = NSi.Get_RotMatrix();
    Sophus::SO3 Ri = NSi.Get_R();
    Vector3d dBgi = NSi.Get_dBias_Gyr();
    Vector3d dBai = NSi.Get_dBias_Acc();

    // terms need to computer error in vertex j, except for bias error
    const NavState& NSj = vj->estimate();
    Vector3d Pj = NSj.Get_P();
    Vector3d Vj = NSj.Get_V();
    //Matrix3d Rj = NSj.Get_RotMatrix();
    Sophus::SO3 Rj = NSj.Get_R();

    // IMU Preintegration measurement
    const IMUPreintegrator& M = _measurement;
    double dTij = M.getDeltaTime();   // Delta Time
    double dT2 = dTij*dTij;
    Vector3d dPij = M.getDeltaP();    // Delta Position pre-integration measurement
    Vector3d dVij = M.getDeltaV();    // Delta Velocity pre-integration measurement
    //Matrix3d dRij = M.getDeltaR();    // Delta Rotation pre-integration measurement
    Sophus::SO3 dRij = Sophus::SO3(M.getDeltaR());

    // tmp variable, transpose of Ri
    //Matrix3d RiT = Ri.transpose();
    Sophus::SO3 RiT = Ri.inverse();
    // residual error of Delta Position measurement
    Vector3d rPij = RiT*(Pj - Pi - Vi*dTij - 0.5*GravityVec*dT2)
                  - (dPij + M.getJPBiasg()*dBgi + M.getJPBiasa()*dBai);   // this line includes correction term of bias change.
    // residual error of Delta Velocity measurement
    Vector3d rVij = RiT*(Vj - Vi - GravityVec*dTij)
                  - (dVij + M.getJVBiasg()*dBgi + M.getJVBiasa()*dBai);   //this line includes correction term of bias change
    // residual error of Delta Rotation measurement
    //Matrix3d dR_dbg = Sophus::SO3::exp(M.Get_J_Phi_Biasg()*dBgi).matrix();
    //Matrix3d rRij = (dRij * dR_dbg).transpose() * Ri.transpose()*Rj;
    //Vector3d rPhiij = Sophus::SO3(rRij).log();
    Sophus::SO3 dR_dbg = Sophus::SO3::exp(M.getJRBiasg()*dBgi);
    Sophus::SO3 rRij = (dRij * dR_dbg).inverse() * RiT * Rj;
    Vector3d rPhiij = rRij.log();

    // residual error of Gyroscope's bias, Forster 15'RSS
    Vector3d rBiasG = (NSj.Get_BiasGyr() + NSj.Get_dBias_Gyr())
                    - (NSi.Get_BiasGyr() + NSi.Get_dBias_Gyr());

    // residual error of Accelerometer's bias, Forster 15'RSS
    Vector3d rBiasA = (NSj.Get_BiasAcc() + NSj.Get_dBias_Acc())
                    - (NSi.Get_BiasAcc() + NSi.Get_dBias_Acc());

    Vector15d err;  // typedef Matrix<double, D, 1> ErrorVector; ErrorVector _error; D=15
    err.setZero();
    // 15-Dim error vector order:
    // position-velocity-rotation-deltabiasGyr_i-deltabiasAcc_i
    // rPij - rVij - rPhiij - rBiasGi - rBiasAi
    err.segment<3>(0) = rPij;       // position error
    err.segment<3>(3) = rVij;       // velocity error
    err.segment<3>(6) = rPhiij;     // rotation phi error
    err.segment<3>(9) = rBiasG;     // bias gyro error
    err.segment<3>(12) = rBiasA;    // bias acc error

    _error = err;

    // Debug log
    //std::cout<<"ns err: "<<_error.transpose()<<std::endl;
}


/**
 * @brief EdgeNavStatePointXYZ::EdgeNavStatePointXYZ
 */
EdgeNavStatePointXYZ::EdgeNavStatePointXYZ() : BaseBinaryEdge<2, Vector2d, VertexSBAPointXYZ, VertexNavState>()
{
}
// Todo
bool EdgeNavStatePointXYZ::read(std::istream& is) {return true;}
bool EdgeNavStatePointXYZ::write(std::ostream& os) const {return true;}

/**
 * @brief VertexGyrBias::VertexGyrBias
 */
VertexGyrBias::VertexGyrBias() : BaseVertex<3, Vector3d>()
{
}

bool VertexGyrBias::read(std::istream& is)
{
    Vector3d est;
    for (int i=0; i<3; i++)
        is  >> est[i];
    setEstimate(est);
    return true;
}

bool VertexGyrBias::write(std::ostream& os) const
{
    Vector3d est(estimate());
    for (int i=0; i<3; i++)
        os << est[i] << " ";
    return os.good();
}

void VertexGyrBias::oplusImpl(const double* update_)  {
    Eigen::Map<const Vector3d> update(update_);
    _estimate += update;
    // Debug log
    //std::cout<<"updated bias estimate: "<<_estimate.transpose()<<", gyr bias update: "<<update.transpose()<<std::endl;
}

/**
 * @brief EdgeGyrBias::EdgeGyrBias
 */
EdgeGyrBias::EdgeGyrBias() : BaseUnaryEdge<3, Vector3d, VertexGyrBias>()
{
}


bool EdgeGyrBias::read(std::istream& is)
{
    return true;
}

bool EdgeGyrBias::write(std::ostream& os) const
{
    return true;
}

void EdgeGyrBias::computeError()
{
    const VertexGyrBias* v = static_cast<const VertexGyrBias*>(_vertices[0]);
    Vector3d bg = v->estimate();
    Matrix3d dRbg = Sophus::SO3::exp(J_dR_bg * bg).matrix();
    Sophus::SO3 errR ( ( dRbij * dRbg ).transpose() * Rwbi.transpose() * Rwbj ); // dRij^T * Riw * Rwj
    _error = errR.log();
    // Debug log
    //std::cout<<"dRbg: "<<std::endl<<dRbg<<std::endl;
    //std::cout<<"error: "<<_error.transpose()<<std::endl;
    //std::cout<<"chi2: "<<_error.dot(information()*_error)<<std::endl;
}

}
