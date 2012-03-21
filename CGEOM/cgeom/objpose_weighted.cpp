#include <cgeom/objpose.h>

#include <ceigen.h>

#include <eigen3/Eigen/Cholesky>
#include <Eigen/LU>
#include <eigen3/Eigen/SVD>

#include <iostream>
#include <numeric>

using namespace std;

////////////////////////////////////////////////////////////////////////////////
double calculate_obj_space_error( const Eigen::MatrixXd& mQ, 
                                  const vector<Eigen::Matrix3d>& vV,
                                  const Eigen::VectorXd& mW ) {
    double dError = 0;
    const int nNumPoints = mQ.cols();
    Eigen::Vector3d mE;
    for( int ii=0; ii<nNumPoints; ii++ ) {
        mE = ( Eigen::Matrix3d::Identity() - vV[ii] )*mQ.col(ii);
        dError += mW[ii] * mE.dot( mE );
    }
    return dError;
}

////////////////////////////////////////////////////////////////////////////////
Eigen::VectorXd calculate_residuals( const Eigen::MatrixXd& mQ, 
                                     const vector<Eigen::Matrix3d>& vV,
                                     const Eigen::VectorXd& mW ) {
    const int nNumPoints = mQ.cols();
    Eigen::VectorXd vR( nNumPoints );
    Eigen::Vector3d mE;
    for( int ii=0; ii<nNumPoints; ii++ ) {
        mE = ( Eigen::Matrix3d::Identity() - vV[ii] )*mQ.col(ii);
        vR[ii] = sqrt( mW[ii] * mE.dot( mE ) );
    }
    return vR;
}

////////////////////////////////////////////////////////////////////////////////
inline Eigen::Vector3d optimal_weighted_t
( const Eigen::MatrixXd& mP3DNotC,
  const vector<Eigen::Matrix3d>& vV,
  const Eigen::Matrix3d& mR,
  const Eigen::VectorXd& mW
  ) {
    Eigen::Matrix3d mtFactor = Eigen::Matrix3d::Zero();
    Eigen::Vector3d vSumFort = Eigen::Vector3d::Zero();
    const int nNumPoints = mP3DNotC.cols();
    for( int ii=0; ii<nNumPoints; ii++ ) {
        const Eigen::Matrix3d mWD = 
            mW[ii] * ( vV[ii] - Eigen::Matrix3d::Identity() );
        mtFactor += mWD;
        vSumFort += mWD * mR * mP3DNotC.col(ii);
    }
    return - mtFactor.inverse() * vSumFort;
}

////////////////////////////////////////////////////////////////////////////////
inline Eigen::Vector3d mean( const Eigen::MatrixXd& mP3DNotC, 
                             const Eigen::VectorXd& mW ) {
    Eigen::Vector3d vMeanP3D = Eigen::Vector3d::Zero();
    double dTotalW = 0;
    for( int ii=0; ii<mP3DNotC.cols(); ii++ ) {
        vMeanP3D += mW[ii] * mP3DNotC.col(ii);
        dTotalW += mW[ii];
    }
    vMeanP3D /= dTotalW;
    return vMeanP3D;
}

////////////////////////////////////////////////////////////////////////////////
inline Eigen::MatrixXd center( const Eigen::MatrixXd& mP3DNotC, 
                               const Eigen::VectorXd& mW ) {
    Eigen::Vector3d vMeanP3D = mean( mP3DNotC, mW );
    Eigen::MatrixXd mP3D = mP3DNotC;
    mP3D.colwise() -= vMeanP3D;
    return mP3D;
}

////////////////////////////////////////////////////////////////////////////////
/// Weighted absolute orientation problem
inline Eigen::Matrix3d optimal_weighted_R
( const Eigen::MatrixXd& mP3DNotC,
  const Eigen::MatrixXd& mQNotC, //<Input/Output
  const Eigen::VectorXd& mW
  ) {
    const int nNumPoints = mP3DNotC.cols();
    // Center 3D points
    Eigen::MatrixXd mP3D = center( mP3DNotC, mW );
    Eigen::MatrixXd mQ   = center( mQNotC, mW );

    Eigen::Matrix3d mM = Eigen::Matrix3d::Zero();
    for( int ii=0; ii<nNumPoints; ii++ ) {
        mM += mW[ii] * mP3D.col(ii) * mQ.col(ii).transpose();
    }
    Eigen::JacobiSVD<Eigen::MatrixXd> svdOfM( mM, Eigen::ComputeThinU | Eigen::ComputeThinV );
    Eigen::Matrix3d mU = svdOfM.matrixU();
    Eigen::Matrix3d mV = svdOfM.matrixV();
    if( mU.diagonal().prod() < 0 ) {
        mU.col(2) = -mU.col(2);
    }
    if( mV.diagonal().prod() < 0 ) {
        mV.col(2) = -mV.col(2);
    }
    return mV * mU.transpose();
}

////////////////////////////////////////////////////////////////////////////////
Eigen::VectorXd compute_weights( const double dExpectedNoiseStd,
                                 const double dError,                     
                                 const Eigen::MatrixXd& mQ, ///<Input
                                 const vector<Eigen::Matrix3d>& vV, ///<Input
                                 vector<bool>& vInliers ///<Input/Output
                                 ) {
    const int nNumParams = 6;
    const int nNumPoints = mQ.cols();
    assert( nNumPoints == vV.size() );
    vInliers.reserve( nNumPoints );
    const int nNumInliers = accumulate( vInliers.begin(), vInliers.end(), int(0) );
    const double dMADFact = max( min( 1., dExpectedNoiseStd/sqrt(dError/nNumInliers) ), 1/40. );
    Eigen::VectorXd mW = Eigen::VectorXd::Ones( nNumPoints ) ;
    mW = CEIGEN::compute_weights( calculate_residuals( mQ, vV, mW ), 
                                  nNumParams, dMADFact, vInliers );
    return mW;
}

////////////////////////////////////////////////////////////////////////////////
void abskernel_weighted
( const Eigen::MatrixXd& mP3DNotC, 
  const vector<Eigen::Matrix3d>& vV,
  const double dOldError,
  const double dExpectedNoiseStd,
  int& nNumInliers, ///<Input/Output
  vector<bool>& vInliers, ///<Output
  Eigen::MatrixXd& mQ, ///<Input/Output
  Eigen::VectorXd& mW, ///<Input/Output
  Eigen::Matrix3d& mR, 
  Eigen::Vector3d& vt, 
  double& dNewError ) {
    const int nNumPoints = mP3DNotC.cols();
    const int nNumParams = 6;

    for( int ii=0; ii<nNumPoints; ii++ ) {
        mQ.col(ii) = vV[ii] * mQ.col(ii);
    }
    mR = optimal_weighted_R( mP3DNotC, mQ, mW );
    vt = optimal_weighted_t( mP3DNotC, vV, mR, mW );

    mQ = mR * mP3DNotC; mQ.colwise() += vt;
    dNewError = calculate_obj_space_error( mQ, vV, mW );

    mW = compute_weights( dExpectedNoiseStd, dOldError,
                          mQ, vV, vInliers );

    nNumInliers = accumulate( vInliers.begin(), vInliers.end(), int(0) );
    // Compute error and compensate for weights to simplify
    // error 'tracking'
    double dWError = calculate_obj_space_error( mQ, vV, mW );
    //cout << "dNewError, dWError: " << dNewError << ", " << dWError << endl;
    mW *= dNewError/dWError;
    //cout << "Check equal to dError: " <<
    //    calculate_obj_space_error( mQ, vV, mW ) << endl;
}

////////////////////////////////////////////////////////////////////////////////
void CGEOM::objpose_robust
( const Eigen::MatrixXd& mP3DNotC, ///<Input: 3xN matrix representing the landmarks in front of the camera
  const Eigen::MatrixXd& mMeas,///<Input: 3xN measurements in the normalised plane or unit sphere
  const int nMaxNumIters,
  const double dTol,
  const double dEpsilon,
  const double dExpectedNoiseStd,
  Eigen::Matrix3d& mR,///<Input/Output: initial rotation estimate, this will also contain the estimated rotation
  Eigen::Vector3d& vt,///Output: estimated translation
  int& nNumIterations,
  double& dObjError,
  vector<bool>& vInliers,
  bool bUseRForInitialisation,
  bool bUsetForInitialisation ) {
    assert( mP3DNotC.rows() == 3 );
    assert( mMeas.rows() == 2 || mMeas.rows() == 3 );
    const int nNumPoints = mP3DNotC.cols();
    if( nNumPoints < 3 ) {
        cout << "WARNING: objpose, not enough points." << endl;
        return;
    }

    const Eigen::Matrix3d O_33 = Eigen::Matrix3d::Zero();
    const int nNumParams = 6;

    // Matrix containing weights
    Eigen::VectorXd mW = Eigen::VectorXd::Ones( nNumPoints ) ;

    // Compute 'projection' matrices
    // V_j = v_j v_j'/ |v_j|^2
    vector<Eigen::Matrix3d> vV; vV.reserve( nNumPoints );
    Eigen::Vector3d vN;
    Eigen::Matrix3d vSumV = O_33;

    if( mMeas.rows() == 2 ) {
        for( int ii=0; ii<nNumPoints; ii++ ) {
            vN.segment<2>(0) = mMeas.col(ii);
            vN(2) = 1;
            vV.push_back
                ( ( vN*vN.transpose() )/ ( vN.transpose()*vN ) );
            vSumV += vV.back();
        }
    }
    else if( mMeas.rows() == 3 ) {
        for( int ii=0; ii<nNumPoints; ii++ ) {
            vN = mMeas.col(ii);
            vN /= mMeas(2,ii); //could also normalise...
            vV.push_back
                ( ( vN*vN.transpose() )/ ( vN.transpose()*vN ) );
            vSumV += vV.back();
        }
    }
    else {
        cerr << "ERROR: in objpose, measurement matrix has wrong number of rows: " << mMeas.rows() << endl;
        return;
    }


    nNumIterations = 0;
    double dOldError = 0.;
    int nNumInliers = nNumPoints/2; // breaking point

    Eigen::MatrixXd mQ( 3, nNumPoints );
    if( bUseRForInitialisation ) {
        // Compute t from R
        if( !bUsetForInitialisation ) {
            vt = optimal_weighted_t( mP3DNotC, vV, mR, mW );
            //PRINT_MATRIX( vt );
        }
        mQ = mR * mP3DNotC; mQ.colwise() += vt;

        mW = compute_weights( dExpectedNoiseStd, dOldError,
                              mQ, vV, vInliers );

        nNumInliers = accumulate( vInliers.begin(), vInliers.end(), int(0) );
        vt = optimal_weighted_t( mP3DNotC, vV, mR, mW ); // 'unbiased' estimate
        //PRINT_MATRIX( vt );
        mQ = mR * mP3DNotC; mQ.colwise() += vt;
        dOldError = 
            calculate_obj_space_error( mQ, vV, mW );
    }
    else {
        cerr << "ERROR: not tested" << endl;
        return;
        mQ = mMeas;
        abskernel_weighted( mP3DNotC, vV,
                            1000.,
                            dExpectedNoiseStd,
                            nNumInliers,
                            vInliers,
                            mQ, //input/output
                            mW, //input/output
                            mR, vt, dOldError );
        nNumIterations = 1;
    }

    double dNewError = 0;
    abskernel_weighted( mP3DNotC, vV,
                        dOldError,
                        dExpectedNoiseStd,
                        nNumInliers,
                        vInliers,
                        mQ, //input/output
                        mW, //input/output
                        mR, vt, dNewError );
    ++nNumIterations;

    while( nNumIterations < nMaxNumIters &&
           abs( (dOldError - dNewError )/dOldError ) > dTol &&
           dNewError > dEpsilon ) {
        dOldError = dNewError;

        abskernel_weighted( mP3DNotC, vV,
                            dOldError,
                            dExpectedNoiseStd,
                            nNumInliers,
                            vInliers,
                            mQ, //input/output
                            mW, //input/output
                            mR, vt, dNewError );

        ++nNumIterations;
    }

    dObjError = sqrt( dNewError/nNumPoints );

    if( mR.diagonal().prod() < 0 ) {
        mR = -mR; vt = -vt;
    }
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
