#include <cgeom/CGeom.h>

#include <ceigen.h>

#include <eigen3/Eigen/Cholesky>
#include <Eigen/LU>
#include <eigen3/Eigen/SVD>
#include <eigen3/unsupported/Eigen/MatrixFunctions>

#include <iostream>
#include <tr1/random>

using namespace std;

namespace CGEOM {
////////////////////////////////////////////////////////////////////////////////
#ifdef EIGEN_DEFAULT_TO_ROW_MAJOR
    // Row major
    const double aHG1[] = {0,0,1,0,0,0,0,0,0};
    const double aHG2[] = {0,0,0,0,0,1,0,0,0};
    const double aHG3[] = {0,1,0,-1,0,0,0,0,0};
    const double aHG4[] = {1/3,0,0,0,1/3,0,0,0,-2/3};
    const double aHG5[] = {1,0,0,0,-1,0,0,0,0};
    const double aHG6[] = {0,1,0,0,0,0,0,0,0};
    const double aHG7[] = {0,0,0,0,0,0,1,0,0};
    const double aHG8[] = {0,0,0,0,0,0,0,1,0};
#else
    const double aHG1[] = {0,0,0,0,0,0,1,0,0};
    const double aHG2[] = {0,0,0,0,0,0,0,1,0};
    const double aHG3[] = {0,-1,0,1,0,0,0,0,0};
    const double aHG4[] = {1./3.,0,0,0,1./3.,0,0,0,-2./3.};
    //const double aHG4[] = {0.333333333,0,0,0,0.33333333,0,0,0,-0.666666666};
    const double aHG5[] = {1,0,0,0,-1,0,0,0,0};
    const double aHG6[] = {0,0,0,1,0,0,0,0,0};
    const double aHG7[] = {0,0,1,0,0,0,0,0,0};
    const double aHG8[] = {0,0,0,0,0,1,0,0,0};
#endif
    const Eigen::Matrix3d aHGs[] = 
        { 
            Eigen::Matrix3d( aHG1 ),
            Eigen::Matrix3d( aHG2 ),
            Eigen::Matrix3d( aHG3 ),
            Eigen::Matrix3d( aHG4 ),
            Eigen::Matrix3d( aHG5 ),
            Eigen::Matrix3d( aHG6 ),
            Eigen::Matrix3d( aHG7 ),
            Eigen::Matrix3d( aHG8 )
        };
}

////////////////////////////////////////////////////////////////////////////////
/// Undistort using the division model (low-level call)
inline void undistort_div_low( const double dLambda, 
                               const double dDX, const double dDY,
                               double& dUX, double& dUY,
                               double& dNormSq,
                               double& dInvDenom ) {
    dNormSq   = dDX*dDX + dDY*dDY;
    dInvDenom = 1 / ( 1 + dLambda*dNormSq );
    dUX = dDX * dInvDenom;
    dUY = dDY * dInvDenom;
}

////////////////////////////////////////////////////////////////////////////////
/// Undistort using the division model
inline void undistort_div( const double dLambda, ///< Input: distortion parameter (0: no distortion) 
                           const double dDX, ///< Input: x pixel value of distorted pixel (obtained from the image)
                           const double dDY, ///< Input: y pixel value of distorted pixel (obtained from the image)
                           double& dUX, ///< Output: x pixel value of distorted pixel (standard projective geometry is valid on these values)
                           double& dUY  ///< Output: y pixel value of distorted pixel  (standard projective geometry is valid on these values)
                           ) {
    double dNormSq; double dInvDenom;
    undistort_div_low( dLambda, dDX, dDY, dUX, dUY, dNormSq, dInvDenom );
}

////////////////////////////////////////////////////////////////////////////////
/// Undistort using the division model
inline void undistort_div( const double dLambda, ///< Input: distortion parameter (0: no distortion) 
                           const Eigen::MatrixXd& mDX, ///< Input: x pixel value of distorted pixel (obtained from the image)
                           Eigen::MatrixXd& mUX ///< Output: x pixel value of distorted pixel (standard projective geometry is valid on these values)
                           ) {
    assert( mDX.cols() == mUX.cols() );
    assert( mDX.rows() >= 2 );
    assert( mUX.rows() >= 2 );
    for( int ii=0; ii<mDX.cols(); ii++ ) {
        undistort_div( dLambda, mDX(0,ii), mDX(1,ii), mUX(0,ii), mUX(1,ii) );
    }
}

////////////////////////////////////////////////////////////////////////////////
/// Undistort using the division model and compute the Jacobian
inline void undistort_div( const double dLambda, ///< Input: distortion parameter (0: no distortion) 
                           const double dDX, ///< Input: x pixel value of distorted pixel (obtained from the image)
                           const double dDY, ///< Input: y pixel value of distorted pixel (obtained from the image)
                           double& dUX,  ///< Output: x pixel value of distorted pixel (standard projective geometry is valid on these values)
                           double& dUY,  ///< Output: y pixel value of distorted pixel  (standard projective geometry is valid on these values)
                           double& dJUX, ///< Output: Jacobian of the undistortion function in lambda for the undistorted x pixel value
                           double& dJUY  ///< Output: Jacobian of the undistortion function in lambda for the undistorted y pixel value
                           ) {
    double dNormSq; double dInvDenom;
    undistort_div_low( dLambda, dDX, dDY, dUX, dUY, dNormSq, dInvDenom );
    dJUX = -dNormSq*dDX * dInvDenom;
    dJUY = -dNormSq*dDY * dInvDenom;
}

////////////////////////////////////////////////////////////////////////////////
/// Projective transfer using a homography
inline void transfer( const Eigen::Matrix3d& mH,
                      const double dUX1,
                      const double dUY1,
                      double& dUX2,
                      double& dUY2,
                      double& dUZ2
                      ) {
    dUX2 = mH(0,0)*dUX1 + mH(0,1)*dUY1 + mH(0,2);
    dUY2 = mH(1,0)*dUX1 + mH(1,1)*dUY1 + mH(1,2);
    dUZ2 = mH(2,0)*dUX1 + mH(2,1)*dUY1 + mH(2,2);
}

////////////////////////////////////////////////////////////////////////////////
/// Transfer using a homography
inline void transfer( const Eigen::Matrix3d& mH,
                      const double dUX1,
                      const double dUY1,
                      double& dUX2,
                      double& dUY2
                      ) {
    double dUZ2;
    transfer( mH, dUX1, dUY1, dUX2, dUY2, dUZ2 );
    double dUZ2Inv = 1/dUZ2;
    dUX2 *= dUZ2Inv;
    dUY2 *= dUZ2Inv;
}

////////////////////////////////////////////////////////////////////////////////
/// Transfer using a homography and compute the 2x8 related Jacobian
inline void transfer( const Eigen::Matrix3d& mH,
                      const double dUX1,
                      const double dUY1,
                      double& dUX2,
                      double& dUY2,
                      Eigen::MatrixXd& mJacH
                      ) {
    double dUZ2;
    transfer( mH, dUX1, dUY1, dUX2, dUY2, dUZ2 );
    const double dUZ2Inv = 1/dUZ2;
    Eigen::Matrix<double,2,3> JProj;
    JProj << 
        dUZ2Inv, 0, -dUX2*dUZ2Inv*dUZ2Inv,
        0, dUZ2Inv, -dUY2*dUZ2Inv*dUZ2Inv;
    Eigen::Matrix<double,3,1> mUX1;
    mUX1 << dUX1, dUY1, 1;
    Eigen::Matrix<double,3,3> mG;
    for( int ii=0; ii<8; ii++ ) {
        mJacH.block(0,ii,2,1) = JProj * mH * CGEOM::aHGs[ii] * mUX1;
    }
    dUX2 *= dUZ2Inv;
    dUY2 *= dUZ2Inv;
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
template<typename Derived>
Eigen::Matrix3d CGEOM::ComputeHomography4Points( const Eigen::MatrixBase<Derived>& mC,
                                                 const double dBoxSize
                                                 ) {
    Eigen::Matrix3d H;
    H(0,0) = (mC(0,0)*mC(0,2)*mC(1,1)-mC(0,1)*mC(1,0)*mC(0,2)-mC(0,0)*mC(1,1)*mC(0,3)+mC(0,1)*mC(1,0)*mC(0,3)-mC(0,0)*mC(0,2)*mC(1,3)+mC(0,0)*mC(0,3)*mC(1,2)+mC(0,1)*mC(0,2)*mC(1,3)-mC(0,1)*mC(0,3)*mC(1,2))/(mC(0,1)*mC(1,2)-mC(0,2)*mC(1,1)-mC(0,1)*mC(1,3)+mC(1,1)*mC(0,3)+mC(0,2)*mC(1,3)-mC(0,3)*mC(1,2));
    H(0,1) = -(mC(0,0)*mC(0,1)*mC(1,2)-mC(0,0)*mC(0,2)*mC(1,1)-mC(0,0)*mC(0,1)*mC(1,3)+mC(0,1)*mC(1,0)*mC(0,3)+mC(0,0)*mC(0,2)*mC(1,3)-mC(1,0)*mC(0,2)*mC(0,3)-mC(0,1)*mC(0,3)*mC(1,2)+mC(0,2)*mC(1,1)*mC(0,3))/(mC(0,1)*mC(1,2)-mC(0,2)*mC(1,1)-mC(0,1)*mC(1,3)+mC(1,1)*mC(0,3)+mC(0,2)*mC(1,3)-mC(0,3)*mC(1,2));
    H(0,2) = mC(0,0);
    H(1,0) = (mC(0,0)*mC(1,1)*mC(1,2)-mC(0,1)*mC(1,0)*mC(1,2)-mC(0,0)*mC(1,1)*mC(1,3)+mC(0,1)*mC(1,0)*mC(1,3)-mC(1,0)*mC(0,2)*mC(1,3)+mC(1,0)*mC(0,3)*mC(1,2)+mC(0,2)*mC(1,1)*mC(1,3)-mC(1,1)*mC(0,3)*mC(1,2))/(mC(0,1)*mC(1,2)-mC(0,2)*mC(1,1)-mC(0,1)*mC(1,3)+mC(1,1)*mC(0,3)+mC(0,2)*mC(1,3)-mC(0,3)*mC(1,2));
    H(1,1) = -(mC(0,1)*mC(1,0)*mC(1,2)-mC(1,0)*mC(0,2)*mC(1,1)-mC(0,0)*mC(1,1)*mC(1,3)+mC(1,0)*mC(1,1)*mC(0,3)+mC(0,0)*mC(1,2)*mC(1,3)-mC(1,0)*mC(0,3)*mC(1,2)-mC(0,1)*mC(1,2)*mC(1,3)+mC(0,2)*mC(1,1)*mC(1,3))/(mC(0,1)*mC(1,2)-mC(0,2)*mC(1,1)-mC(0,1)*mC(1,3)+mC(1,1)*mC(0,3)+mC(0,2)*mC(1,3)-mC(0,3)*mC(1,2));
    H(1,2) = mC(1,0);
    H(2,0) = (mC(0,0)*mC(1,2)-mC(1,0)*mC(0,2)-mC(0,0)*mC(1,3)+mC(1,0)*mC(0,3)+mC(0,2)*mC(1,3)-mC(0,3)*mC(1,2))/(mC(0,1)*mC(1,2)-mC(0,2)*mC(1,1)-mC(0,1)*mC(1,3)+mC(1,1)*mC(0,3)+mC(0,2)*mC(1,3)-mC(0,3)*mC(1,2))-1.0;
    H(2,1) = (mC(0,0)*mC(1,1)-mC(0,1)*mC(1,0)-mC(0,0)*mC(1,2)+mC(1,0)*mC(0,2)+mC(0,1)*mC(1,2)-mC(0,2)*mC(1,1))/(mC(0,1)*mC(1,2)-mC(0,2)*mC(1,1)-mC(0,1)*mC(1,3)+mC(1,1)*mC(0,3)+mC(0,2)*mC(1,3)-mC(0,3)*mC(1,2))-1.0;
    H(2,2) = 1.0;
    H.block(0,0,3,2) /= (dBoxSize-1);
    return H;
}

////////////////////////////////////////////////////////////////////////////////
template<typename Derived1, typename Derived2>
Eigen::Matrix3d CGEOM::ComputeHomography4Points( const Eigen::MatrixBase<Derived1>& mC1,
                                                 const Eigen::MatrixBase<Derived2>& mC2
                                                 ) {
    Eigen::Matrix3d H;
    Eigen::Matrix<double,8,8> mA = Eigen::Matrix<double,8,8>::Zero();
    Eigen::Matrix<double,8,1> mb;

    mA(0,1) = -mC1(0,0);
    mA(0,2) = mC1(0,0)*mC2(1,0);
    mA(0,4) = -mC1(1,0);
    mA(0,5) = mC1(1,0)*mC2(1,0);
    mA(0,7) = -1.0;
    mA(1,0) = mC1(0,0);
    mA(1,2) = -mC1(0,0)*mC2(0,0);
    mA(1,3) = mC1(1,0);
    mA(1,5) = -mC1(1,0)*mC2(0,0);
    mA(1,6) = 1.0;
    mA(2,1) = -mC1(0,1);
    mA(2,2) = mC1(0,1)*mC2(1,1);
    mA(2,4) = -mC1(1,1);
    mA(2,5) = mC1(1,1)*mC2(1,1);
    mA(2,7) = -1.0;
    mA(3,0) = mC1(0,1);
    mA(3,2) = -mC1(0,1)*mC2(0,1);
    mA(3,3) = mC1(1,1);
    mA(3,5) = -mC1(1,1)*mC2(0,1);
    mA(3,6) = 1.0;
    mA(4,1) = -mC1(0,2);
    mA(4,2) = mC1(0,2)*mC2(1,2);
    mA(4,4) = -mC1(1,2);
    mA(4,5) = mC1(1,2)*mC2(1,2);
    mA(4,7) = -1.0;
    mA(5,0) = mC1(0,2);
    mA(5,2) = -mC1(0,2)*mC2(0,2);
    mA(5,3) = mC1(1,2);
    mA(5,5) = -mC1(1,2)*mC2(0,2);
    mA(5,6) = 1.0;
    mA(6,1) = -mC1(0,3);
    mA(6,2) = mC1(0,3)*mC2(1,3);
    mA(6,4) = -mC1(1,3);
    mA(6,5) = mC1(1,3)*mC2(1,3);
    mA(6,7) = -1.0;
    mA(7,0) = mC1(0,3);
    mA(7,2) = -mC1(0,3)*mC2(0,3);
    mA(7,3) = mC1(1,3);
    mA(7,5) = -mC1(1,3)*mC2(0,3);
    mA(7,6) = 1.0;

    mb(0,0) = mC2(1,0);
    mb(1,0) = -mC2(0,0);
    mb(2,0) = mC2(1,1);
    mb(3,0) = -mC2(0,1);
    mb(4,0) = mC2(1,2);
    mb(5,0) = -mC2(0,2);
    mb(6,0) = mC2(1,3);
    mb(7,0) = -mC2(0,3);

    Eigen::Matrix<double,8,1> mH = mA.lu().solve( -mb );
    for( int ii=0; ii<8; ii++ ) {
        H(ii) = mH(ii);
    }
    H(2,2) = 1;
    return H;
}

////////////////////////////////////////////////////////////////////////////////
void CGEOM::ComputeHomographyDistortionJacobians
(  const Eigen::MatrixXd& mPoints1, // 2xN or 3xN
   const Eigen::MatrixXd& mPoints2, // 2xN or 3xN
   const Eigen::Matrix3d& mH,
   const double dLambda,
   Eigen::Matrix<double,9,9>& mJtJ,
   Eigen::Matrix<double,9,1>& mJtf,
   double& dSumSquaredErrors
   ) {
    Eigen::Matrix<double,2,1> mE;
    dSumSquaredErrors = 0;
    mJtJ = Eigen::Matrix<double,9,9>::Zero();
    mJtf = Eigen::Matrix<double,9,1>::Zero();
    Eigen::MatrixXd mJTransfer( 2, 9 );
    for( int jj=0; jj<mPoints1.cols(); jj++ ) {
        // Undistort
        double dUX1, dUY1, dJUX1, dJUY1;
        double dUX2, dUY2, dJUX2, dJUY2;
        undistort_div( dLambda, 
                       mPoints1(0,jj), mPoints1(1,jj),
                       dUX1, dUY1, dJUX1, dJUY1
                       );
        undistort_div( dLambda, 
                       mPoints2(0,jj), mPoints2(1,jj),
                       dUX2, dUY2, dJUX2, dJUY2
                       );
                
        double dUX2Est, dUY2Est;
        transfer( mH, dUX1, dUY1, dUX2Est, dUY2Est,
                  mJTransfer );

        // Jacobian for Lambda
        mJTransfer(0,8) = 
            ( mH(0,0) - mH(2,0) * dUX1 ) * dJUX1 +
            ( mH(0,1) - mH(2,1) * dUX1 ) * dJUY1 - dJUX2;
        mJTransfer(1,8) = 
            ( mH(1,0) - mH(2,0) * dUY1 ) * dJUX1 +
            ( mH(1,1) - mH(2,1) * dUY1 ) * dJUY1 - dJUY2;

        // Error vector
        mE(0,0) = dUX2Est - dUX2;
        mE(1,0) = dUY2Est - dUY2;

        mJtJ += mJTransfer.transpose() * mJTransfer;
        mJtf += mJTransfer.transpose() * mE;

        dSumSquaredErrors += mE.squaredNorm();
    }
}

////////////////////////////////////////////////////////////////////////////////
void CGEOM::ComputeHomographyJacobians
(  const Eigen::MatrixXd& mPoints1, // 2xN or 3xN
   const Eigen::MatrixXd& mPoints2, // 2xN or 3xN
   const Eigen::Matrix3d& mH,
   Eigen::Matrix<double,8,8>& mJtJ,
   Eigen::Matrix<double,8,1>& mJtf,
   double& dSumSquaredErrors,
   const std::vector<short int>& vInliers
   ) {
    Eigen::Matrix<double,2,1> mE;
    dSumSquaredErrors = 0;
    mJtJ = Eigen::Matrix<double,8,8>::Zero();
    mJtf = Eigen::Matrix<double,8,1>::Zero();
    Eigen::MatrixXd mJTransfer( 2, 8 );
    for( size_t ii=0; ii<vInliers.size(); ii++ ) {
        int jj = vInliers[ii];
        double dX1 = mPoints1(0,jj);
        double dY1 = mPoints1(1,jj);
        double dX2Est, dY2Est;
        transfer( mH, dX1, dY1, dX2Est, dY2Est,
                  mJTransfer );

        // Error vector
        mE(0,0) = dX2Est - mPoints2(0,jj);
        mE(1,0) = dY2Est - mPoints2(1,jj);

        mJtJ += mJTransfer.transpose() * mJTransfer;
        mJtf += mJTransfer.transpose() * mE;

        dSumSquaredErrors += mE.squaredNorm();
    }
}

////////////////////////////////////////////////////////////////////////////////
void CGEOM::ComputeHomographyDistortion( const Eigen::MatrixXd& mPoints1, // 2xN or 3xN
                                  const Eigen::MatrixXd& mPoints2, // 2xN or 3xN
                                  Eigen::Matrix3d& mH,
                                  double& dLambda
                                  )
{
    assert( mPoints1.cols() == mPoints2.cols() );
    const int nMaxNumIters = 10;
    const double dMinDiff = 1e-4;
    double dNu = 2.;

    double dSumSquaredErrors = 0;
        
    Eigen::Matrix3d mHNew;
    double dLambdaNew = 0;
    Eigen::Matrix<double,9,9> mJtJ, mJtJNew;
    Eigen::Matrix<double,9,1> mJtf, mJtfNew;        
        
    Eigen::Matrix<double,9,1> vUpdate;
    Eigen::Matrix3d mHUpdate;

    ComputeHomographyDistortionJacobians
        ( mPoints1, mPoints2, mH, dLambda,
          mJtJ, mJtf, dSumSquaredErrors );
    double dRMS = sqrt( dSumSquaredErrors/mPoints1.cols() );
    double dNewRMS;
    std::cout << "RMS: " << dRMS << std::endl;

    double dMu = 1e-3 * mJtJ.lpNorm<Eigen::Infinity>();

    for( int ii=0; ii<nMaxNumIters; ii++ ) {
        // Compute the update by solving the normal equations
        mJtJ += dMu * Eigen::Matrix<double,9,9>::Identity();
        vUpdate = mJtJ.ldlt().solve( mJtf );
        vUpdate = -vUpdate;
        mHUpdate = Eigen::Matrix3d::Zero();
        for( int ii=0; ii<8; ii++ ) {
            mHUpdate += vUpdate(ii,0) * aHGs[ii];
        }
        mHNew = mH * ( Eigen::Matrix3d::Identity() + mHUpdate + 0.5*mHUpdate*mHUpdate ); //approx to exponential
        dLambdaNew = dLambdaNew + vUpdate(8,0);

        ComputeHomographyDistortionJacobians
            ( mPoints1, mPoints2, mHNew, dLambdaNew,
              mJtJNew, mJtfNew, dSumSquaredErrors );
        dNewRMS = sqrt( dSumSquaredErrors/mPoints1.cols() );

        if( dNewRMS < dRMS ) {
            dMu *= 0.333333;
            dNu = 2.;
            double dRMSDiff = dRMS - dNewRMS;
            mH      = mHNew;
            dLambda = dLambdaNew;
            mJtJ    = mJtJNew;
            mJtf    = mJtfNew;
            dRMS    = dNewRMS;
            std::cout << "RMS: " << dRMS << std::endl;

            if( dRMSDiff < dMinDiff ) {
                return;
            }
        }
        else {
            dMu *= dNu;
            dNu *= 2;
        }
    }
}

////////////////////////////////////////////////////////////////////////////////
void CGEOM::ComputeHomographyNonLin( const Eigen::MatrixXd& mPoints1, // 2xN or 3xN
                              const Eigen::MatrixXd& mPoints2, // 2xN or 3xN
                              Eigen::Matrix3d& mH,
                              double& dRMS,
                              const std::vector<short int>& vInliers
                              )
{
    assert( mPoints1.cols() == mPoints2.cols() );
    const int nMaxNumIters = 10;
    const double dMinDiff = 1e-4;
    double dNu = 2.;

    double dSumSquaredErrors = 0;
        
    Eigen::Matrix3d mHNew;
    Eigen::Matrix<double,8,8> mJtJ, mJtJNew;
    Eigen::Matrix<double,8,1> mJtf, mJtfNew;        
        
    Eigen::Matrix<double,8,1> vUpdate;
    Eigen::Matrix3d mHUpdate;

    ComputeHomographyJacobians
        ( mPoints1, mPoints2, mH, 
          mJtJ, mJtf, dSumSquaredErrors,
          vInliers );
    dRMS = sqrt( dSumSquaredErrors/mPoints1.cols() );
    double dNewRMS;
    //std::cout << "RMS: " << dRMS << std::endl;

    double dMu = 1e-3 * mJtJ.lpNorm<Eigen::Infinity>();

    for( int ii=0; ii<nMaxNumIters; ii++ ) {
        // Compute the update by solving the normal equations
        mJtJ += dMu * Eigen::Matrix<double,8,8>::Identity();
        vUpdate = mJtJ.ldlt().solve( mJtf );
        vUpdate = -vUpdate;
        mHUpdate = Eigen::Matrix3d::Zero();
        for( int ii=0; ii<8; ii++ ) {
            mHUpdate += vUpdate(ii,0) * aHGs[ii];
        }
        mHNew = mH * ( Eigen::Matrix3d::Identity() + mHUpdate + 0.5*mHUpdate*mHUpdate ); //approx to exponential

        ComputeHomographyJacobians
            ( mPoints1, mPoints2, mHNew, 
              mJtJNew, mJtfNew, dSumSquaredErrors,
              vInliers );
        dNewRMS = sqrt( dSumSquaredErrors/mPoints1.cols() );

        if( dNewRMS < dRMS ) {
            dMu *= 0.333333;
            dNu = 2.;
            double dRMSDiff = dRMS - dNewRMS;
            mH      = mHNew;
            mJtJ    = mJtJNew;
            mJtf    = mJtfNew;
            dRMS    = dNewRMS;
            //std::cout << "RMS: " << dRMS << std::endl;

            if( dRMSDiff < dMinDiff ) {
                return;
            }
        }
        else {
            dMu *= dNu;
            dNu *= 2;
        }
    }
}

////////////////////////////////////////////////////////////////////////////////
// Geometry
void CGEOM::AddHomographyConstraint( Eigen::MatrixXd& HConst, 
                                     const int nIndex, // point index, defines the place where to add constrain i HConst
                                     const double dX1, const double dY1,
                                     const double dX2, const double dY2
                                     )
{
    if( 3*( nIndex + 1 ) > HConst.rows() ) {
        std::cerr << "WARNING: in AddHomographyConstraint, constraint matrix HConst is too small." << std::endl;
        return;
    }
    const int nRow = 3*nIndex;

    HConst( nRow, 0 ) = 0;
    HConst( nRow, 1 ) = 0;
    HConst( nRow, 2 ) = 0;
    HConst( nRow, 3 ) = -dX1;
    HConst( nRow, 4 ) = -dY1;
    HConst( nRow, 5 ) = -1;
    HConst( nRow, 6 ) = dY2*dX1;
    HConst( nRow, 7 ) = dY2*dY1;
    HConst( nRow, 8 ) = dY2;

    HConst( nRow+1, 0 ) = dX1;
    HConst( nRow+1, 1 ) = dY1;
    HConst( nRow+1, 2 ) = 1;
    HConst( nRow+1, 3 ) = 0;
    HConst( nRow+1, 4 ) = 0;
    HConst( nRow+1, 5 ) = 0;
    HConst( nRow+1, 6 ) = -dX2*dX1;
    HConst( nRow+1, 7 ) = -dX2*dY1;
    HConst( nRow+1, 8 ) = -dX2;

    HConst( nRow+2, 0 ) = -dY2*dX1;
    HConst( nRow+2, 1 ) = -dY2*dY1;
    HConst( nRow+2, 2 ) = -dY2;
    HConst( nRow+2, 3 ) = dX2*dX1;
    HConst( nRow+2, 4 ) = dX2*dY1;
    HConst( nRow+2, 5 ) = dX2;
    HConst( nRow+2, 6 ) = 0;
    HConst( nRow+2, 7 ) = 0;
    HConst( nRow+2, 8 ) = 0;
}

////////////////////////////////////////////////////////////////////////////////
Eigen::Matrix3d CGEOM::ComputeHomography( const Eigen::MatrixXd& mPoints1, // 2xN or 3xN
                                          const Eigen::MatrixXd& mPoints2  // 2xN or 3xN
                                          )
{
    assert( mPoints1.cols() == mPoints2.cols() );

    Eigen::Matrix3d mH;
    Eigen::MatrixXd HConst( 3*mPoints1.cols(), 9 ); // Matrix of constraints
    for( int ii=0; ii<mPoints1.cols(); ii++ ) {
        AddHomographyConstraint( HConst, ii,
                                 mPoints1( 0, ii ), mPoints1( 1, ii ), 
                                 mPoints2( 0, ii ), mPoints2( 1, ii ) );
    }
    //std::cout << std::endl << HConst << std::endl << std::endl;

    // Solve
    Eigen::JacobiSVD<Eigen::MatrixXd> svdOfHConst( HConst, Eigen::ComputeThinV ); 
    //svdOfHConst.compute( HConst );
    Eigen::MatrixXd V = svdOfHConst.matrixV();
    mH = Eigen::Map<Eigen::MatrixXd>( V.block(0,8,9,1).data(), 3, 3 ).transpose();
    return mH;
}

////////////////////////////////////////////////////////////////////////////////
Eigen::Matrix3d CGEOM::ComputeHomography( const Eigen::MatrixXd& mPoints1, // 2xN or 3xN
                                          const Eigen::MatrixXd& mPoints2, // 2xN or 3xN
                                          const std::vector<short  int>& vInlierIndeces )
{
    assert( mPoints1.cols() == mPoints2.cols() );

    Eigen::Matrix3d mH;
    Eigen::MatrixXd HConst( 3*vInlierIndeces.size(), 9 ); // Matrix of constraints
    for( size_t ii=0; ii<vInlierIndeces.size(); ii++ ) {
        AddHomographyConstraint( HConst, ii,
                                 mPoints1( 0, vInlierIndeces[ii] ), 
                                 mPoints1( 1, vInlierIndeces[ii] ), 
                                 mPoints2( 0, vInlierIndeces[ii] ), 
                                 mPoints2( 1, vInlierIndeces[ii] ) );
    }
    //std::cout << std::endl << HConst << std::endl << std::endl;

    // Solve
    Eigen::JacobiSVD<Eigen::MatrixXd> svdOfHConst( HConst, Eigen::ComputeThinV ); 
    //svdOfHConst.compute( HConst );
    Eigen::MatrixXd V = svdOfHConst.matrixV();
    mH = Eigen::Map<Eigen::MatrixXd>( V.block(0,8,9,1).data(), 3, 3 ).transpose();
    return mH;
}

////////////////////////////////////////////////////////////////////////////////
bool CGEOM::ransac_homography( const int nMaxNumTrials, const double dInlierThresh,
                               const Eigen::MatrixXd& mPoints1, // 2xN or 3xN
                               const Eigen::MatrixXd& mPoints2, // 2xN or 3xN
                               Eigen::Matrix3d& mHBest,
                               std::vector<short int>& vBestInlierIndeces,
                               double& dBestInlierError ) 
{

    if( mPoints1.cols() != mPoints2.cols() ) {
        std::cerr << "ERROR: point matrices do not have the same number of columns." << std::endl;
        return false;
    }
    if( mPoints1.cols() < 4 ) {
        std::cerr << "ERROR: point matrices do not have the enough points." << std::endl;
        return false;
    }

    std::tr1::mt19937 eng; 
    std::tr1::uniform_int<int> unif( 0, mPoints1.cols()-1 );

    Eigen::Matrix<double,2,4> mRandomPoints1( 2, 4 );
    Eigen::Matrix<double,2,4> mRandomPoints2( 2, 4 );
    Eigen::Matrix<double, 1, Eigen::Dynamic> mSquaredError( 1, mPoints1.cols() );

    Eigen::Matrix3d mH;

    dBestInlierError = std::numeric_limits<double>::max();
    int nBestNumInliers = 0;
    double dInlierError;
    std::vector<short int> vInlierIndeces;
    const double dInlierThreshSquared = dInlierThresh * dInlierThresh;
    for( int nTrial=0; nTrial<nMaxNumTrials; nTrial++ ) {
        // Pick four points
        int n1, n2, n3, n4;
        n1 = unif( eng );
        do {
            n2 = unif( eng );
        } while( n2 == n1 );
        do {
            n3 = unif( eng );
        } while( n3 == n1 || n3 == n2 );
        do {
            n4 = unif( eng );
        } while( n4 == n1 || n4 == n2 || n4 == n3  );
  
        mRandomPoints1.col( 0 ) = mPoints1.col( n1 );
        mRandomPoints1.col( 1 ) = mPoints1.col( n2 );
        mRandomPoints1.col( 2 ) = mPoints1.col( n3 );
        mRandomPoints1.col( 3 ) = mPoints1.col( n4 );

        mRandomPoints2.col( 0 ) = mPoints2.col( n1 );
        mRandomPoints2.col( 1 ) = mPoints2.col( n2 );
        mRandomPoints2.col( 2 ) = mPoints2.col( n3 );
        mRandomPoints2.col( 3 ) = mPoints2.col( n4 );

        // Compute homography
        mH = ComputeHomography( mRandomPoints1, mRandomPoints2 );

        // Compute error for all points
        mSquaredError = ( mPoints2 - CEIGEN::metric( mH*CEIGEN::projective( mPoints1 ) ) ).colwise().squaredNorm();
            
        //std::cout << mH << std::endl;
        //std::cout << mSquaredError << std::endl;

        // Compute inlier error (MSAC)
        dInlierError = 0;
        vInlierIndeces.clear();
        for( int ii=0; ii<mSquaredError.cols(); ii++ ) {
            if( mSquaredError[ii] < dInlierThreshSquared ) {
                dInlierError += mSquaredError[ii];
                vInlierIndeces.push_back( ii );
            }
        }
        if( vInlierIndeces.size() < 4 ) {
            //std::cerr << "WARNING: fewer than 4 inliers!!!" << std::endl;
            continue;
        }

#if 1
        // For testing (expensive)
        mH = ComputeHomography( mPoints1, mPoints2, vInlierIndeces ); 
        mSquaredError = ( mPoints2 - CEIGEN::metric( mH*CEIGEN::projective( mPoints1 ) ) ).colwise().squaredNorm();
        dInlierError = 0;
        vInlierIndeces.clear();
        for( int ii=0; ii<mSquaredError.cols(); ii++ ) {
            if( mSquaredError[ii] < dInlierThreshSquared ) {
                dInlierError += mSquaredError[ii];
                vInlierIndeces.push_back( ii );
            }
        }
            

#endif

        if( vInlierIndeces.size() < 4 ) {
            //std::cerr << "WARNING: fewer than 4 inliers!!!" << std::endl;
            continue;
        }

        //if( dInlierError < dBestInlierError ) {
        if( nBestNumInliers < (int)vInlierIndeces.size() ) {
            nBestNumInliers    = vInlierIndeces.size();
            dBestInlierError   = dInlierError;
            vBestInlierIndeces = vInlierIndeces;

#if 0
            std::cout << n1 << " " << n2 << " " << n3 << " " << n4 << std::endl;
            for( int ii=0; ii<mSquaredError.cols(); ii++ ) {
                std::cout << sqrt( mSquaredError[ii] ) << " ";
            }
            std::cout << std::endl;
            for( int ii=0; ii<vBestInlierIndeces.size(); ii++ ) {
                std::cout <<  vBestInlierIndeces[ ii ] << " ";
            }
            std::cout << std::endl;
#endif
        }
    }

    mHBest = ComputeHomography( mPoints1, mPoints2, vBestInlierIndeces );

    Eigen::Matrix3d mHNonLin = mHBest;
    double dRMS;
    ComputeHomographyNonLin( mPoints1, mPoints2, mHNonLin,
                             dRMS, vBestInlierIndeces );

    mSquaredError = ( mPoints2 - CEIGEN::metric( mHBest*CEIGEN::projective( mPoints1 ) ) ).colwise().squaredNorm();
    dInlierError = 0;
    vInlierIndeces.clear();
    for( int ii=0; ii<mSquaredError.cols(); ii++ ) {
        if( mSquaredError[ii] < dInlierThreshSquared ) {
            dInlierError += mSquaredError[ii];
            vInlierIndeces.push_back( ii );
        }
    }        
    std::cout << "dInlierError: " << dInlierError << std::endl;

    mSquaredError = ( mPoints2 - CEIGEN::metric( mHNonLin*CEIGEN::projective( mPoints1 ) ) ).colwise().squaredNorm();
    dInlierError = 0;
    vInlierIndeces.clear();
    for( int ii=0; ii<mSquaredError.cols(); ii++ ) {
        if( mSquaredError[ii] < dInlierThreshSquared ) {
            dInlierError += mSquaredError[ii];
            vInlierIndeces.push_back( ii );
        }
    }        
    std::cout << "dInlierError: " << dInlierError << std::endl;

    return true;
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
static void apply_update( const Eigen::VectorXd& vUpdate,
                          Eigen::Matrix3d& mR,
                          Eigen::Vector3d& vt
                          ) {
    mR *= ( CEIGEN::skew_rot<Eigen::Matrix3d>(vUpdate[0],0,0) +
            CEIGEN::skew_rot<Eigen::Matrix3d>(0,vUpdate[1],0) +
            CEIGEN::skew_rot<Eigen::Matrix3d>(0,0,vUpdate[2]) ).exp();
    vt += vUpdate.segment<3>(3);
}

//////////////////////////////////////////////////////////////////////////////// 
static void trans_proj( const Eigen::Vector3d& mP3D,
                        const Eigen::Matrix3d& mR, 
                        const Eigen::Vector3d& vt,
                        Eigen::Vector2d& mTransProjP3D ) {
    const Eigen::Vector3d mTransP3D = mR * mP3D + vt;
    mTransProjP3D = mTransP3D.segment(0,2) / mTransP3D(2);
}

//////////////////////////////////////////////////////////////////////////////// 
static void trans_proj( const Eigen::Vector3d& mP3D,
                        const Eigen::Matrix3d& mR, 
                        const Eigen::Vector3d& vt,
                        Eigen::Vector2d& mTransProjP3D,
                        Eigen::Matrix<double,2,6>& mdmProjP3DdRt ///<Output: Jacobian
                        ) {
    const int nNumParams = 6;
    const Eigen::Vector3d mTransP3D = mR * mP3D + vt;
    mTransProjP3D = mTransP3D.segment(0,2) / mTransP3D(2);
#if 0
    // Finite difference
    Eigen::Vector2d mTransProjP3D_eps;
    Eigen::Matrix3d mR_eps; 
    Eigen::Vector3d vt_eps;
    Eigen::Matrix<double,6,1> vUpdate;
    double dEps = 1e-6;
    for( int ii=0; ii<6; ii++ ) {
        mR_eps = mR; vt_eps = vt;
        vUpdate = Eigen::Matrix<double,6,1>::Zero();
        vUpdate(ii) += dEps;
        apply_update( vUpdate, mR_eps, vt_eps );
        trans_proj( mP3D, mR_eps, vt_eps, mTransProjP3D_eps );
        mdmProjP3DdRt.col(ii) << 
            ( mTransProjP3D_eps(0) - mTransProjP3D(0) )/dEps,
            ( mTransProjP3D_eps(1) - mTransProjP3D(1) )/dEps;            
    }

    //PRINT_MATRIX( mdmProjP3DdRt );
    return;
#endif

    Eigen::Matrix<double,2,3> mdmProjdP;
    const double dInvPz = 1./mTransP3D(2);
    mdmProjdP << 
        dInvPz, 0., -mTransP3D(0)*dInvPz*dInvPz,
        0., dInvPz, -mTransP3D(1)*dInvPz*dInvPz;
    
    Eigen::Matrix<double,3,nNumParams> mdRtX;
    // Rotation part
    mdRtX.block(0,0,3,1) = mR*CEIGEN::skew_rot<Eigen::Matrix3d>(1,0,0)*mP3D;
    mdRtX.block(0,1,3,1) = mR*CEIGEN::skew_rot<Eigen::Matrix3d>(0,1,0)*mP3D;
    mdRtX.block(0,2,3,1) = mR*CEIGEN::skew_rot<Eigen::Matrix3d>(0,0,1)*mP3D;
    // Translation part
    mdRtX.block(0,3,3,3) = Eigen::Matrix3d::Identity();

    mdmProjP3DdRt = mdmProjdP * mdRtX;
    //PRINT_MATRIX( mdmProjP3DdRt );   
    return;
}

//////////////////////////////////////////////////////////////////////////////// 
static void compute_update_extrinsic
( const Eigen::Matrix3d& mR, 
  const Eigen::Vector3d& vt,
  const Eigen::MatrixXd& m3D,
  const Eigen::MatrixXd& mMeas,
  const double dStabiliser,
  Eigen::VectorXd& vUpdate, ///<Output: least-squares parameter update
  double& dRMS,     ///<Output: reprojection error
  double& dRMSGain, ///<Output: predicted gain from the linearisation
  double& dGradMagn ///<Output: magnitude of the least-squares Jacobian
  ) {
    typedef unsigned int uint;
    const int nNumPoints = m3D.cols();
    assert( nNumPoints == mMeas.cols() );
    const int nNumParams  = 6;
    Eigen::MatrixXd mJtJ = dStabiliser*Eigen::MatrixXd::Identity( nNumParams, nNumParams );
    Eigen::VectorXd mJte = Eigen::VectorXd::Zero( nNumParams );

    dRMS = 0.;
    Eigen::Vector2d vProj3D;
    Eigen::Vector2d vE;
    Eigen::Matrix<double,2,nNumParams> mdmProjP3DdRt;
    for( int nPI = 0; nPI<nNumPoints; nPI++ ) {
        trans_proj( m3D.col(nPI), mR, vt,
                    vProj3D, mdmProjP3DdRt );
        vE = vProj3D - mMeas.col(nPI).segment(0,2);

        mJtJ += mdmProjP3DdRt.transpose() * mdmProjP3DdRt;
        mJte += mdmProjP3DdRt.transpose() * vE;

        dRMS += vE.transpose() * vE;
    }
    
    dRMS = sqrt( dRMS/nNumPoints );

    vUpdate = Eigen::VectorXd::Zero( nNumParams );
    Eigen::JacobiSVD<Eigen::MatrixXd> svd( mJtJ + dStabiliser*Eigen::MatrixXd::Identity(mJtJ.rows(),mJtJ.cols()),
                                           Eigen::ComputeThinU | Eigen::ComputeThinV );
    vUpdate   = -svd.solve( mJte );
    dRMSGain  = vUpdate.transpose() * ( vUpdate - mJte );
    dRMSGain  = sqrt( dRMSGain / nNumPoints );
    dGradMagn = mJte.lpNorm<Eigen::Infinity>();
}

////////////////////////////////////////////////////////////////////////////////
void CGEOM::imgpose( const Eigen::MatrixXd& mP3D, ///<Input: 3xN matrix representing the landmarks in front of the camera
                     const Eigen::MatrixXd& mMeas,///<Input: 3xN measurements in the normalised plane or unit sphere
                     const int nMaxNumIters,
                     const double dTol,
                     const double dEpsilon,
                     Eigen::Matrix3d& mR,///<Input/Output: initial rotation estimate, this will also contain the estimated rotation
                     Eigen::Vector3d& vt,///Output: estimated translation
                     int& nNumIterations,
                     double& dImgError,
                     bool bUseRtForInitialisation ) {

    const int nNumParams = 6;
    const int nNumPoints = mP3D.cols();
    if( nNumPoints == 0 ) {
        return;
    }
    if( !bUseRtForInitialisation ) {
        mR = Eigen::Matrix3d::Identity();
        vt = Eigen::Vector3d::Zero();
    }

    double dRMS = 0, dRMSGain = 0, dGradMagn = 0;
    double dStabiliser = 1;
    Eigen::VectorXd vUpdate;

    double dPrevRMS = 0;
    const double dRMSThresh = 1e-3;
    const double dRMSGainThresh = 1e-10;

    Eigen::Matrix3d mR_prev = mR;
    Eigen::Vector3d vt_prev = vt;
    double dRMSPrev;
    compute_update_extrinsic( mR, vt, mP3D, mMeas,
                              dStabiliser, vUpdate, dRMSPrev, 
                              dRMSGain, dGradMagn );

    for( int nIter=0; nIter<nMaxNumIters; nIter++ ) {
 
        apply_update( vUpdate, mR, vt );
        compute_update_extrinsic( mR, vt, mP3D, mMeas,
                                  dStabiliser, vUpdate, dRMS, dRMSGain, dGradMagn );
        //PRINT_SCALAR( dRMS );

        if( dRMS < dRMSPrev ) {
            mR_prev = mR;
            vt_prev = vt;
            dRMSPrev = dRMS;
            dStabiliser /= 1.5;
        }
        else {
            mR = mR_prev;
            vt_prev = vt;
            dStabiliser *= 2;
            vUpdate = Eigen::VectorXd::Zero(6);
        }

        if( abs( dRMS - dPrevRMS ) < dRMSThresh &&
            dRMSGain < dRMSGainThresh ) {
            break;
        }
    }

    dImgError = dRMS;
    return;
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
#define INSTANTIATE( T ) \
    template                                                            \
    Eigen::Matrix3d CGEOM::ComputeHomography4Points( const Eigen::MatrixBase<T>&, \
                                                     const double       \
                                                     );                 

#define INSTANTIATE2( T1, T2 ) \
    template                                                            \
    Eigen::Matrix3d CGEOM::ComputeHomography4Points( const Eigen::MatrixBase<T1>&, \
                                                     const Eigen::MatrixBase<T2>& \
                                                     );                 \

typedef Eigen::Matrix<float,2,4> Matrix24f;
typedef Eigen::Matrix<double,2,4> Matrix24d;

INSTANTIATE( Matrix24f );
INSTANTIATE( Matrix24d );
INSTANTIATE( Eigen::MatrixXf );
INSTANTIATE( Eigen::MatrixXd );

INSTANTIATE2( Matrix24f, Matrix24f);
INSTANTIATE2( Matrix24f, Matrix24d );
INSTANTIATE2( Matrix24f, Eigen::MatrixXf );
INSTANTIATE2( Matrix24f, Eigen::MatrixXd );
INSTANTIATE2( Matrix24d, Matrix24f);
INSTANTIATE2( Matrix24d, Matrix24d );
INSTANTIATE2( Matrix24d, Eigen::MatrixXf );
INSTANTIATE2( Matrix24d, Eigen::MatrixXd );
INSTANTIATE2( Eigen::MatrixXf, Matrix24f);
INSTANTIATE2( Eigen::MatrixXf, Matrix24d );
INSTANTIATE2( Eigen::MatrixXf, Eigen::MatrixXf );
INSTANTIATE2( Eigen::MatrixXf, Eigen::MatrixXd );
INSTANTIATE2( Eigen::MatrixXd, Matrix24f);
INSTANTIATE2( Eigen::MatrixXd, Matrix24d );
INSTANTIATE2( Eigen::MatrixXd, Eigen::MatrixXf );
INSTANTIATE2( Eigen::MatrixXd, Eigen::MatrixXd );

