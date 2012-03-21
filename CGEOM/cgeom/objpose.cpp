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
                                  const vector<Eigen::Matrix3d>& vV ) {
    double dError = 0;
    const int nNumPoints = mQ.cols();
    Eigen::Vector3d mE;
    for( int ii=0; ii<nNumPoints; ii++ ) {
        mE = ( Eigen::Matrix3d::Identity() - vV[ii] )*mQ.col(ii);
        dError += mE.dot( mE );
    }
    return dError;
}

////////////////////////////////////////////////////////////////////////////////
inline Eigen::Vector3d optimal_t( const Eigen::MatrixXd& mP3DCentered,
                                  const vector<Eigen::Matrix3d>& vV,
                                  const Eigen::Matrix3d& mFactFort,
                                  const Eigen::Matrix3d& mR ) {
    assert( mP3DCentered.rowwise().mean()(0) > -1e-6 &&
            mP3DCentered.rowwise().mean()(0) < 1e-6 &&
            mP3DCentered.rowwise().mean()(1) > -1e-6 &&
            mP3DCentered.rowwise().mean()(1) < 1e-6 );
    Eigen::Vector3d vSumFort = Eigen::Vector3d::Zero();
    const int nNumPoints = mP3DCentered.cols();
    for( int ii=0; ii<nNumPoints; ii++ ) {
        vSumFort += vV[ii] *mR*mP3DCentered.col(ii);
    }
    return mFactFort * vSumFort;
}

////////////////////////////////////////////////////////////////////////////////
/// Absolute orientation problem
inline Eigen::Matrix3d optimal_R( const Eigen::MatrixXd& mP3DCentered,
                                  Eigen::MatrixXd& mQ //<Input/Output
                                  ) {
    assert( mP3DCentered.rowwise().mean()(0) > -1e-6 &&
            mP3DCentered.rowwise().mean()(0) < 1e-6 &&
            mP3DCentered.rowwise().mean()(1) > -1e-6 &&
            mP3DCentered.rowwise().mean()(1) < 1e-6 );

    const int nNumPoints = mP3DCentered.cols();
    // Center 3D points
    Eigen::Vector3d vMeanQ = mQ.rowwise().mean();
    mQ.colwise() -= vMeanQ;

    Eigen::Matrix3d mM = Eigen::Matrix3d::Zero();
    for( int ii=0; ii<nNumPoints; ii++ ) {
        mM += mP3DCentered.col(ii) * mQ.col(ii).transpose();
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
void abskernel( const Eigen::MatrixXd& mP3DCentered, 
                const vector<Eigen::Matrix3d>& vV,
                const Eigen::Matrix3d& mFactFort, 
                Eigen::MatrixXd& mQ, ///<Input/Output
                Eigen::Matrix3d& mR, 
                Eigen::Vector3d& vt, 
                double& dNewError ) {
    const int nNumPoints = mP3DCentered.cols();
    for( int ii=0; ii<nNumPoints; ii++ ) {
        mQ.col(ii) = vV[ii] * mQ.col(ii);
    }
    mR = optimal_R( mP3DCentered, mQ );
    vt = optimal_t( mP3DCentered, vV, mFactFort, mR );

    mQ = mR * mP3DCentered; mQ.colwise() += vt;
    dNewError = calculate_obj_space_error( mQ, vV );
}

////////////////////////////////////////////////////////////////////////////////
void CGEOM::objpose( const Eigen::MatrixXd& mP3DNotC, ///<Input: 3xN matrix representing the landmarks in front of the camera
                     const Eigen::MatrixXd& mMeas,///<Input: 3xN measurements in the normalised plane or unit sphere
                     const int nMaxNumIters,
                     const double dTol,
                     const double dEpsilon,
                     Eigen::Matrix3d& mR,///<Input/Output: initial rotation estimate, this will also contain the estimated rotation
                     Eigen::Vector3d& vt,///Output: estimated translation
                     int& nNumIterations,
                     double& dObjError,
                     bool bUseRtForInitialisation ) {
    assert( mP3DNotC.rows() == 3 );
    assert( mMeas.rows() == 2 || mMeas.rows() == 3 );
    const int nNumPoints = mP3DNotC.cols();
    const double dNumPointsInv = 1./double(nNumPoints);
    if( nNumPoints < 3 ) {
        cout << "WARNING: objpose, not enough points." << endl;
        return;
    }

    const Eigen::Matrix3d I_33 = Eigen::Matrix3d::Identity();
    const Eigen::Matrix3d O_33 = Eigen::Matrix3d::Zero();
    const Eigen::Vector3d O_3 = Eigen::Vector3d::Zero();

    // Center 3D points
    Eigen::MatrixXd mP3D = mP3DNotC;
    Eigen::Vector3d vMeanP3D =
        mP3D.rowwise().mean();
    mP3D.colwise() -= vMeanP3D;

    // Compute 'projection' matrices
    // V_j = 
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
    
    // Compute ( I - 1/n \sum V_I )^{-1}
    // This will be used to compute the values for the translation
    Eigen::Matrix3d mFactFort = 
        dNumPointsInv *( I_33 - dNumPointsInv * vSumV ).inverse();

    nNumIterations = 0;
    double dOldError = 0.;
    Eigen::MatrixXd mQ( 3, mP3D.cols() );
    if( bUseRtForInitialisation ) {
        // Compute t from R
        vt = optimal_t( mP3D, vV, mFactFort, mR );
        //PRINT_MATRIX( vt );
        mQ = mR * mP3D; mQ.colwise() += vt;
        dOldError = 
            calculate_obj_space_error( mQ, vV );
    }
    else {
        mQ = mMeas;
        abskernel( mP3D, vV, mFactFort,
                   mQ, //input/output
                   mR, vt, dOldError );
        nNumIterations = 1;
    }

    double dNewError = 0;
    abskernel( mP3D, vV, mFactFort,
               mQ, //input/output
               mR, vt, dNewError );

#if 0
    PRINT_SCALAR( dOldError );
    PRINT_SCALAR( dNewError );
    
    PRINT_MATRIX( mR );
    PRINT_MATRIX( vt + mR*vMeanP3D ); 
    PRINT_MATRIX( vt - mR*vMeanP3D ); 
#endif

    ++nNumIterations;

    while( nNumIterations < nMaxNumIters &&
           abs( (dOldError - dNewError )/dOldError ) > dTol &&
           dNewError > dEpsilon ) {
        dOldError = dNewError;

        abskernel( mP3D, vV, mFactFort,
                   mQ, //input/output
                   mR, vt, dNewError );

        ++nNumIterations;
    }

    dObjError = sqrt( dNewError/nNumPoints );

    if( mR.diagonal().prod() < 0 ) {
        mR = -mR; vt = -vt;
        vt += mR*vMeanP3D; 
    }
    else {
        vt -= mR*vMeanP3D; 
    }
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
