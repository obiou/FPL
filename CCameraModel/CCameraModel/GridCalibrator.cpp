#include <CCameraModel/CameraModel.h>
#include <CCameraModel/GridCalibrator.h>

#include <ceigen.h>
#include <cgeom/CGeom.h>

#include <eigen3/Eigen/SVD>
#include <eigen3/unsupported/Eigen/MatrixFunctions>

#include <iostream>

////////////////////////////////////////////////////////////////////////////////
bool CCameraModel::compute_extrinsics( const CameraModel& cameraModel,
                                       const Eigen::MatrixXd& mP3D,
                                       const Eigen::MatrixXd& mP2D,
                                       Eigen::Matrix3d& mR,
                                       Eigen::Vector3d& mt ) {
    // Lift image points
    Eigen::MatrixXd mP3DEst( 3, mP3D.cols() );
    Eigen::MatrixXd mP3DEstNorm( 2, mP3D.cols());
    if( !cameraModel.lift( mP2D, mP3DEst ) ) {
        std::cout << "Lifting failed." << std::endl;
        CCameraModel::print_values( cameraModel );
        return false;
    }
    // Compute homography
    mP3DEstNorm = CEIGEN::metric( mP3DEst );
    Eigen::Matrix3d mH = CGEOM::ComputeHomography( mP3D, mP3DEstNorm );
    CEIGEN::HToSE3( mH, mR, mt );
    return true;
}

//////////////////////////////////////////////////////////////////////////////// 
bool CCameraModel::compute_extrinsics( const CameraModel& cameraModel,
                                       const Eigen::MatrixXd& mP3D,
                                       const Eigen::MatrixXd& mP2D,
                                       const std::vector<short int>& vInlierIndeces,
                                       Eigen::Matrix3d& mR,
                                       Eigen::Vector3d& mt) {
    // Lift image points
    Eigen::MatrixXd mP3DEst( 3, mP3D.cols() );
    Eigen::MatrixXd mP3DEstNorm( 2, mP3D.cols());
    if( !cameraModel.lift( mP2D, mP3DEst ) ) {
        std::cout << "Lifting failed." << std::endl;
        CCameraModel::print_values( cameraModel );
        return false;
    }
    // Compute homography
    mP3DEstNorm = CEIGEN::metric( mP3DEst );
    Eigen::Matrix3d mH = CGEOM::ComputeHomography( mP3D, mP3DEstNorm, vInlierIndeces );
    CEIGEN::HToSE3( mH, mR, mt );
    return true;
}

//////////////////////////////////////////////////////////////////////////////// 
bool CCameraModel::compute_extrinsics_non_lin( const CameraModel& cameraModel,
                                               const Eigen::MatrixXd& mP3D,
                                               const Eigen::MatrixXd& mP2D,
                                               Eigen::Matrix3d& mR,
                                               Eigen::Vector3d& mt ) {
    if( !compute_extrinsics( cameraModel, mP3D, mP2D, mR, mt ) ) {
        return false;
    }
    std::vector<Eigen::Matrix3d> vR; vR.push_back( mR );
    std::vector<Eigen::Vector3d> vt; vt.push_back( mt );
    std::vector<Eigen::MatrixXd> vImagePoints;
    vImagePoints.push_back( mP2D );

    double dRMS = 0, dRMSGain = 0, dGradMagn = 0;
    double dStabiliser = 1;
    Eigen::VectorXd vUpdate;

    const int nMaxIter = 10;

    for( int nIter=0; nIter<nMaxIter; nIter++ ) {
        CCameraModel::compute_update_extrinsic( vR, vt, cameraModel, vImagePoints, mP3D, 
                                                dStabiliser, vUpdate, dRMS, dRMSGain, dGradMagn );
        std::cout << dRMS << std::endl;
        CCameraModel::apply_update( vUpdate, vR, vt );
    }

    return true;
}

//////////////////////////////////////////////////////////////////////////////// 
void CCameraModel::compute_update_extrinsic
( const std::vector<Eigen::Matrix3d>& vR, 
  const std::vector<Eigen::Vector3d>& vt,
  const CameraModel& cameraModel, 
  const std::vector<Eigen::MatrixXd>& vImagePoints,
  const Eigen::MatrixXd& mGrid,
  const double dStabiliser,
  Eigen::VectorXd& vUpdate, ///<Output: least-squares parameter update
  double& dRMS,     ///<Output: reprojection error
  double& dRMSGain, ///<Output: predicted gain from the linearisation
  double& dGradMagn ///<Output: magnitude of the least-squares Jacobian
  ) {
    typedef unsigned int uint;
    const uint nNumViews  = vImagePoints.size();
    assert( nNumViews == vR.size() );
    assert( nNumViews == vt.size() );
    if( nNumViews < 1 ) { return; }
    const int nNumPoints      = mGrid.cols();
    const int nNumPoseParams  = 6;
    const int nTotalNumParams = nNumPoseParams*nNumViews;
    Eigen::MatrixXd mJtJ = dStabiliser*Eigen::MatrixXd::Identity( nTotalNumParams, nTotalNumParams );
    Eigen::VectorXd mJte = Eigen::VectorXd::Zero( nTotalNumParams );
    double dSumSqErrors = 0;
    for( size_t ii=0; ii<nNumViews; ii++ ) {
        const size_t nPoseIndex = (unsigned int)nNumPoseParams * ii;
        Eigen::MatrixXd mP2D( 2, nNumPoints );
        Eigen::MatrixXd mdP2DE( 2, nNumPoseParams*nNumPoints );
        cameraModel.project_trans( vR[ii], vt[ii], mGrid, mP2D, mdP2DE );
        Eigen::MatrixXd vE = mP2D - vImagePoints[ii];        
        for( size_t nPI = 0; nPI<(unsigned int)nNumPoints; nPI++ ) {
            Eigen::MatrixXd mdP2DEPart = mdP2DE.block(0,nNumPoseParams*nPI,2,nNumPoseParams);
            mJtJ.block( nPoseIndex,      nPoseIndex,      nNumPoseParams, nNumPoseParams ) += 
                mdP2DEPart.transpose() * mdP2DEPart;
            mJte.segment(nPoseIndex,nNumPoseParams) += mdP2DEPart.transpose() * vE.block(0,nPI,2,1);
        }
        dSumSqErrors += vE.squaredNorm();
    }

    vUpdate = Eigen::VectorXd::Zero( nNumViews*nNumPoseParams );
    Eigen::JacobiSVD<Eigen::MatrixXd> svd( mJtJ, Eigen::ComputeThinU | Eigen::ComputeThinV );
    vUpdate = -svd.solve( mJte );
    dRMS = sqrt( dSumSqErrors / ( nNumPoints*nNumViews ) );
    dRMSGain = vUpdate.transpose() * ( dStabiliser*vUpdate - mJte );
    dRMSGain = sqrt( dRMSGain / ( nNumPoints*nNumViews ) );
    dGradMagn = mJte.lpNorm<Eigen::Infinity>();
}

//////////////////////////////////////////////////////////////////////////////// 
void CCameraModel::compute_update( const std::vector<Eigen::Matrix3d>& vR,
                                   const std::vector<Eigen::Vector3d>& vt,
                                   const CameraModel& cameraModel,
                                   const std::vector<Eigen::MatrixXd>& vImagePoints,
                                   const Eigen::MatrixXd& mGrid,
                                   const double dStabiliser,
                                   Eigen::VectorXd& vUpdate, ///<Output: least-squares parameter update
                                   double& dRMS,     ///<Output: reprojection error
                                   double& dRMSGain, ///<Output: predicted gain from the linearisation
                                   double& dGradMagn,///<Output: magnitude of the least-squares Jacobian
                                   bool bFull ) {
    typedef unsigned int uint;
    const uint nNumViews  = vImagePoints.size();
    assert( nNumViews == vR.size() );
    assert( nNumViews == vt.size() );
    if( nNumViews < 1 ) { return; }
    const int nNumPoints      = mGrid.cols();
    const int nNumCamParams   = cameraModel.get_num_parameters();
    const int nNumPoseParams  = 6;
    const int nTotalNumParams = nNumPoseParams*nNumViews + nNumCamParams;
    Eigen::MatrixXd mJtJ = dStabiliser*Eigen::MatrixXd::Identity( nTotalNumParams, nTotalNumParams );
    Eigen::VectorXd mJte = Eigen::VectorXd::Zero( nTotalNumParams );
    double dSumSqErrors = 0;
    const size_t nCamParamsIndex = nNumPoseParams*nNumViews;
    for( size_t ii=0; ii<nNumViews; ii++ ) {
        const size_t nPoseIndex = (unsigned int)nNumPoseParams * ii;
        Eigen::MatrixXd mP2D( 2, nNumPoints );
        Eigen::MatrixXd mdP2DE( 2, nNumPoseParams*nNumPoints );
        Eigen::MatrixXd mdP2DI( 2, nNumCamParams*nNumPoints );
        cameraModel.project_trans( vR[ii], vt[ii], mGrid, mP2D,
                                   mdP2DE, mdP2DI );
        Eigen::MatrixXd vE = mP2D - vImagePoints[ii];        
        for( size_t nPI = 0; nPI<(unsigned int)nNumPoints; nPI++ ) {
            // Ignore NaN observations (unseen grid points)
            if( isnan(vImagePoints[ii].col(nPI)(0)) || isnan(vImagePoints[ii].col(nPI)(1)) ) {
                vE.col(nPI) << 0 , 0;
            }else{
                Eigen::MatrixXd mdP2DEPart = mdP2DE.block(0,nNumPoseParams*nPI,2,nNumPoseParams);
                Eigen::MatrixXd mdP2DIPart = mdP2DI.block(0,nNumCamParams*nPI,2,nNumCamParams);
                mJtJ.block( nPoseIndex,      nPoseIndex,      nNumPoseParams, nNumPoseParams ) +=
                    mdP2DEPart.transpose() * mdP2DEPart;
                mJtJ.block( nPoseIndex,      nCamParamsIndex, nNumPoseParams, nNumCamParams )  +=
                    mdP2DEPart.transpose() * mdP2DIPart;
                mJtJ.block( nCamParamsIndex, nPoseIndex,      nNumCamParams,  nNumPoseParams )  +=
                    mdP2DIPart.transpose() * mdP2DEPart;
                mJtJ.block( nCamParamsIndex, nCamParamsIndex, nNumCamParams, nNumCamParams )   +=
                    mdP2DIPart.transpose() * mdP2DIPart;

                mJte.segment(nPoseIndex,nNumPoseParams) += mdP2DEPart.transpose() * vE.block(0,nPI,2,1);
                mJte.segment(nCamParamsIndex,nNumCamParams) += mdP2DIPart.transpose() * vE.block(0,nPI,2,1);
            }
        }
        dSumSqErrors += vE.squaredNorm();
    }

    vUpdate = Eigen::VectorXd::Zero( nNumViews*nNumPoseParams + nNumCamParams );
    if( bFull ) {
        Eigen::JacobiSVD<Eigen::MatrixXd> svd( mJtJ, Eigen::ComputeThinU | Eigen::ComputeThinV );
        vUpdate = -svd.solve( mJte );
    }
    else {
        Eigen::JacobiSVD<Eigen::MatrixXd> svd( mJtJ.block( 0, 0, nNumViews*nNumPoseParams, nNumViews*nNumPoseParams ), 
                                               Eigen::ComputeThinU | Eigen::ComputeThinV );
        vUpdate.segment( 0, nNumViews*nNumPoseParams ) = -svd.solve( mJte.segment( 0, nNumViews*nNumPoseParams ) );
    }
    dRMS = sqrt( dSumSqErrors / ( nNumPoints*nNumViews ) );
    dRMSGain = vUpdate.transpose() * ( dStabiliser*vUpdate - mJte );
    dRMSGain = sqrt( dRMSGain / ( nNumPoints*nNumViews ) );
    dGradMagn = mJte.lpNorm<Eigen::Infinity>();
}

////////////////////////////////////////////////////////////////////////////////
void CCameraModel::apply_update( const Eigen::VectorXd& vUpdate,
                                 std::vector<Eigen::Matrix3d>& vR,
                                 std::vector<Eigen::Vector3d>& vt
                                 ) {
    const int nNumPoseParams = 6;
    const unsigned int nNumViews = vR.size();
    for( size_t nView=0; nView<nNumViews; nView++ ) {
        vR[nView] *= ( CEIGEN::skew_rot<Eigen::Matrix3d>(vUpdate[nNumPoseParams*nView+0],0,0) +
                       CEIGEN::skew_rot<Eigen::Matrix3d>(0,vUpdate[nNumPoseParams*nView+1],0) +
                       CEIGEN::skew_rot<Eigen::Matrix3d>(0,0,vUpdate[nNumPoseParams*nView+2]) ).exp();
        vt[nView](0,0) += vUpdate[nNumPoseParams*nView+3];
        vt[nView](1,0) += vUpdate[nNumPoseParams*nView+4];
        vt[nView](2,0) += vUpdate[nNumPoseParams*nView+5];
    }
}

////////////////////////////////////////////////////////////////////////////////
void CCameraModel::apply_update( const Eigen::VectorXd& vUpdate,
                                 std::vector<Eigen::Matrix3d>& vR,
                                 std::vector<Eigen::Vector3d>& vt,
                                 CameraModel& cameraModel
                                 ) {
    apply_update( vUpdate, vR, vt );
    const int nNumPoseParams = 6;
    const unsigned int nNumViews = vR.size();
    const unsigned int nCamIndex = nNumPoseParams*nNumViews;
    for( int ii=0; ii<vUpdate.rows()-nCamIndex; ii++ ) {
        const std::string sVarName = cameraModel.parameter_index_to_name( ii );
        cameraModel.set( sVarName, 
                         cameraModel.get<double>( sVarName ) + vUpdate[nCamIndex + ii] );
    }
}

//////////////////////////////////////////////////////////////////////////////// 
void CCameraModel::print_values( const CameraModel& cameraModel ) {
    for( int ii=0; ii<cameraModel.get_num_parameters(); ii++ ) {
        std::string sParName = cameraModel.parameter_index_to_name( ii );
        std::cout << sParName << ": " << 
            cameraModel.get<double>( sParName ) << std::endl;
    }
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

CCameraModel::GridCalibrator::GridCalibrator( const std::string& sCameraType,
                                              const unsigned int nImageWidth,
                                              const unsigned int nImageHeight,
                                              Eigen::MatrixXd& pattern
                                              ) :
    m_nImageWidth( nImageWidth ), m_nImageHeight( nImageHeight )
{
    m_mGrid = pattern;
    CameraModel* pCameraModel = new CameraModel( sCameraType );
    if( pCameraModel != NULL ) {
        m_CameraModel = *pCameraModel;
        m_CameraModel.initialise_parameters( nImageWidth, nImageHeight );
    }
}


//////////////////////////////////////////////////////////////////////////////// 
CCameraModel::GridCalibrator::GridCalibrator( const std::string& sCameraType,
                                              const unsigned int nImageWidth,
                                              const unsigned int nImageHeight,
                                              const unsigned int nBoardWidth,
                                              const unsigned int nBoardHeight
                                              ) :
    m_nImageWidth( nImageWidth ), m_nImageHeight( nImageHeight )
{
    m_mGrid = CEIGEN::make_grid( nBoardWidth, nBoardHeight );
    CameraModel* pCameraModel = new CameraModel( sCameraType );
    if( pCameraModel != NULL ) {
        m_CameraModel = *pCameraModel;
        m_CameraModel.initialise_parameters( nImageWidth, nImageHeight );
    }
}
        
////////////////////////////////////////////////////////////////////////
/// Add a new view with a set of 2D measurements
/// (internally the extrinsic will be initialised)
void CCameraModel::GridCalibrator::add_view( const Eigen::MatrixXd& mP2D ) {
    m_vImagePoints.push_back( mP2D );

    Eigen::Matrix3d mR;
    Eigen::Vector3d mt;
    if( !compute_extrinsics( m_CameraModel, m_mGrid, m_vImagePoints.back(),
                             mR, mt ) ) {
        m_CameraModel.initialise_parameters( m_nImageWidth, m_nImageHeight );
        if( !compute_extrinsics( m_CameraModel, m_mGrid, m_vImagePoints.back(),
                                 mR, mt ) ) {
            mR = Eigen::Matrix3d::Identity();
            mt = Eigen::Vector3d::Zero();
            mt(2,0) = 1;
        }
    }
    m_vR.push_back( mR );
    m_vt.push_back( mt );
}

////////////////////////////////////////////////////////////////////////
/// Add a new view with a set of 2D measurements
/// (internally the extrinsic will be initialised)
void CCameraModel::GridCalibrator::add_view( const Eigen::MatrixXd& mP2D,
                                             const std::vector<short int>& vInlierIndeces
                                             ) {
    m_vImagePoints.push_back( mP2D );

    Eigen::Matrix3d mR;
    Eigen::Vector3d mt;
    if( !compute_extrinsics( m_CameraModel, m_mGrid, m_vImagePoints.back(),
                             vInlierIndeces, mR, mt ) ) {
        m_CameraModel.initialise_parameters( m_nImageWidth, m_nImageHeight );
        if( !compute_extrinsics( m_CameraModel, m_mGrid, m_vImagePoints.back(),
                                 vInlierIndeces, mR, mt ) ) {
            mR = Eigen::Matrix3d::Identity();
            mt = Eigen::Vector3d::Zero();
            mt(2,0) = 1;
        }
    }
    m_vR.push_back( mR );
    m_vt.push_back( mt );
}

////////////////////////////////////////////////////////////////////////
/// Make one full update step.
/// Returns false if the error increases.
/// TODO avoid bad updates.
bool CCameraModel::GridCalibrator::iterate( double& dRMS ) {
    double dRMSGain = 0, dGradMagn = 0;
    //const double dStabiliser = 0.1;
    const double dStabiliser = 1;
    Eigen::VectorXd vUpdate;
    // Update only extrinsics
    CCameraModel::compute_update( m_vR, m_vt, m_CameraModel, m_vImagePoints, 
                                  m_mGrid, dStabiliser,
                                  vUpdate, dRMS, dRMSGain, dGradMagn,
                                  false );
    CCameraModel::apply_update( vUpdate, m_vR, m_vt, m_CameraModel );
    CCameraModel::compute_update( m_vR, m_vt, m_CameraModel, m_vImagePoints, 
                                  m_mGrid, dStabiliser,
                                  vUpdate, dRMS, dRMSGain, dGradMagn,
                                  true );
    CCameraModel::apply_update( vUpdate, m_vR, m_vt, m_CameraModel );
    m_dPrevRMS = dRMS;
    return m_dPrevRMS <= dRMS;
}

////////////////////////////////////////////////////////////////////////
/// Make a full minimisation
double CCameraModel::GridCalibrator::minimise() {
    if( m_vt.empty() ) { 
        std::cerr << "ERROR in minimise(), empty translation vector." << std::endl;
        return -1;
    }
    double dRMS = 0, dRMSGain = 0, dGradMagn = 0;
    double dStabiliser = 1;
    Eigen::VectorXd vUpdate;

    // Update only extrinsics
    CCameraModel::compute_update( m_vR, m_vt, m_CameraModel, m_vImagePoints, 
                                  m_mGrid, dStabiliser,
                                  vUpdate, dRMS, dRMSGain, dGradMagn,
                                  true );

    const int nMaxNumIters    = 100;
    const double dUpdateTol   = 1e-7;
    const double dGradMagnTol = 1e-7;
    double dPrevRMS = m_dPrevRMS;
    int nIter = 0;
    double dRMSBest = 1e10;
    std::vector<Eigen::Matrix3d> m_vRBest = m_vR;
    std::vector<Eigen::Vector3d> m_vtBest = m_vt;
    std::stringstream ss; ss << m_CameraModel;

    for( nIter=0; nIter<nMaxNumIters; nIter++ ) {
        if( vUpdate.norm() < dUpdateTol ) {
            std::cout << "Exiting due to small update norm." << std::endl;
            break;
        }
        // Compute stabiliser as a function of RMS (we know what the RMS should be at the solution)
        dStabiliser = dRMS > 10 ? 0.1 : std::max( 0.1/8. * (dRMS-2.), 0. );
        CCameraModel::apply_update( vUpdate, m_vR, m_vt, m_CameraModel );
        CCameraModel::compute_update( m_vR, m_vt, m_CameraModel, m_vImagePoints, 
                                      m_mGrid, dStabiliser,
                                      vUpdate, dRMS, dRMSGain, dGradMagn,
                                      true );
        std::cout << "dRMSAccept: " << dRMS << " (dStabiliser: " << dStabiliser << ")." << std::endl;
        if( dRMS < dRMSBest ) {
            m_vRBest = m_vR;
            m_vtBest = m_vt;
            ss << m_CameraModel;
            dRMSBest = dRMS;
        }
       
        if( dGradMagn < dGradMagnTol ) {
            std::cout << "Exiting due to small gradient magnitude." << std::endl;
            break;
        }
        dPrevRMS = dRMS;
    }
    m_vR = m_vRBest;
    m_vt = m_vtBest;
    ss >> m_CameraModel;
    CCameraModel::compute_update( m_vR, m_vt, m_CameraModel, m_vImagePoints, 
                                      m_mGrid, dStabiliser,
                                      vUpdate, dRMS, dRMSGain, dGradMagn,
                                      true );
    std::cout << "Number of iterations: " << nIter << std::endl;
    std::cout << "Final RMS: " << dRMS << std::endl;
    CCameraModel::print_values( m_CameraModel );
    std::cout << m_vt[0] << std::endl;

    return dRMS;
}

////////////////////////////////////////////////////////////////////////
void CCameraModel::GridCalibrator::print() {
    std::cout << m_CameraModel << std::endl;
}

////////////////////////////////////////////////////////////////////////
bool CCameraModel::GridCalibrator::save( const std::string& sFileName ) {
    return m_CameraModel.save( sFileName );
}

////////////////////////////////////////////////////////////////////////
CCameraModel::CameraModel* CCameraModel::GridCalibrator::get_camera_copy() {
    CameraModel* pCam = new CameraModel();
    //std::stringstream oss;
    //oss << m_CameraModel; oss >> *pCam;
    m_CameraModel.save( "/tmp/tmp_calib" );
    pCam->load( "/tmp/tmp_calib" );
    return pCam;
}
