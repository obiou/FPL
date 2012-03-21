#ifndef CGEOM_H
#define CGEOM_H

#include <eigen3/Eigen/Core>

#include <vector>

#define PRINT_MATRIX( M ) std::cout << #M << ": " << std::endl << M << std::endl;
#define PRINT_SCALAR( M ) std::cout << #M << ": " << M << std::endl;

namespace CGEOM {
    ////////////////////////////////////////////////////////////////////////////////
    /// Fast explicit homography computation between a canonical box of size
    /// dBoxSize and 4 points.
    /// Assumes mH(2,2) = 1 (this could be an issue if the plane goes through the optical center)
    /// After computation: mC ~ mH * box
    template<typename Derived>
        Eigen::Matrix3d ComputeHomography4Points( const Eigen::MatrixBase<Derived>& mC,
                                                  const double dBoxSize
                                                  );

    ////////////////////////////////////////////////////////////////////////////////
    /// Fast homography computation using pivoting if we only have 4 point correspondances
    /// Assumes mH(2,2) = 1 (this could be an issue if the plane goes through the optical center)
    /// After computation: mC2 ~ mH * mC1
    template<typename Derived1, typename Derived2>
        Eigen::Matrix3d ComputeHomography4Points( const Eigen::MatrixBase<Derived1>& mC1,
                                                  const Eigen::MatrixBase<Derived2>& mC2
                                                  );

    ////////////////////////////////////////////////////////////////////////////////
    void ComputeHomographyDistortionJacobians
        (  const Eigen::MatrixXd& mPoints1, // 2xN or 3xN
           const Eigen::MatrixXd& mPoints2, // 2xN or 3xN
           const Eigen::Matrix3d& mH,
           const double dLambda,
           Eigen::Matrix<double,9,9>& mJtJ,
           Eigen::Matrix<double,9,1>& mJtf,
           double& dSumSquaredErrors
           );

    ////////////////////////////////////////////////////////////////////////////////
    void ComputeHomographyJacobians
        (  const Eigen::MatrixXd& mPoints1, // 2xN or 3xN
           const Eigen::MatrixXd& mPoints2, // 2xN or 3xN
           const Eigen::Matrix3d& mH,
           Eigen::Matrix<double,8,8>& mJtJ,
           Eigen::Matrix<double,8,1>& mJtf,
           double& dSumSquaredErrors,
           const std::vector<short int>& vInliers
           );

    ////////////////////////////////////////////////////////////////////////////////
    /// Compute the homography and distortion parameter from
    /// associated points
    ///
    /// The homography 'mH' and lambda 'dLambda' are input/output parameters.
    ///
    void ComputeHomographyDistortion( const Eigen::MatrixXd& mPoints1, // 2xN or 3xN
                                      const Eigen::MatrixXd& mPoints2, // 2xN or 3xN
                                      Eigen::Matrix3d& mH,
                                      double& dLambda
                                      );

    ////////////////////////////////////////////////////////////////////////////////
    /// Compute the homography and distortion parameter from
    /// associated points
    ///
    /// The homography 'mH' and lambda 'dLambda' are input/output parameters.
    ///
    void ComputeHomographyNonLin( const Eigen::MatrixXd& mPoints1, // 2xN or 3xN
                                  const Eigen::MatrixXd& mPoints2, // 2xN or 3xN
                                  Eigen::Matrix3d& mH,
                                  double& dRMS,
                                  const std::vector<short int>& vInliers
                                  );
    
    ////////////////////////////////////////////////////////////////////////////////
    // Geometry
    void AddHomographyConstraint( Eigen::MatrixXd& HConst, 
                                  const int nIndex, // point index, defines the place where to add constrain i HConst
                                  const double dX1, const double dY1,
                                  const double dX2, const double dY2
                                  );

    ////////////////////////////////////////////////////////////////////////////////
    /// Will return H such that
    /// mPoints2 = H*mPoints1
    Eigen::Matrix3d ComputeHomography( const Eigen::MatrixXd& mPoints1, // 2xN or 3xN
                                       const Eigen::MatrixXd& mPoints2  // 2xN or 3xN
                                       );

    ////////////////////////////////////////////////////////////////////////////////
    Eigen::Matrix3d ComputeHomography( const Eigen::MatrixXd& mPoints1, // 2xN or 3xN
                                       const Eigen::MatrixXd& mPoints2, // 2xN or 3xN
                                       const std::vector<short  int>& vInlierIndeces );

    ////////////////////////////////////////////////////////////////////////////////
    bool ransac_homography( const int nMaxNumTrials, const double dInlierThresh,
                            const Eigen::MatrixXd& mPoints1, // 2xN or 3xN
                            const Eigen::MatrixXd& mPoints2, // 2xN or 3xN
                            Eigen::Matrix3d& mHBest,
                            std::vector<short int>& vBestInlierIndeces,
                            double& dBestInlierError );

    ////////////////////////////////////////////////////////////////////////////////
    /// Pose estimation from measurement in the image plane.
    /// The convergence region is smaller than 'objpose' but the final precision
    /// is mildy better (as we are minimising the optimal cost function assuming a Gaussian error model).
    void imgpose( const Eigen::MatrixXd& mP3D, ///<Input: 3xN matrix representing the landmarks in front of the camera
                  const Eigen::MatrixXd& mMeas,///<Input: 3xN measurements in the normalised plane or unit sphere
                  const int nMaxNumIters,
                  const double dTol,
                  const double dEpsilon,
                  Eigen::Matrix3d& mR,///<Input/Output: initial rotation estimate, this will also contain the estimated rotation
                  Eigen::Vector3d& vt,///Output: estimated translation
                  int& nNumIterations,
                  double& dImgError,
                  bool bUseRtForInitialisation = false );
}

#endif
