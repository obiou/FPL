#ifndef OBJPOSE_H
#define OBJPOSE_H

#include <eigen3/Eigen/Core>

#include <vector>

#define PRINT_MATRIX( M ) std::cout << #M << ": " << std::endl << M << std::endl;
#define PRINT_SCALAR( M ) std::cout << #M << ": " << M << std::endl;

namespace CGEOM {
    ////////////////////////////////////////////////////////////////////////////////
    /// C++ version of 
    /// "Fast and Globally Convergent Pose Estimation from Video Images" by Chien-Ping Lu et. al, PAMI 98
    void objpose( const Eigen::MatrixXd& mP3D, ///<Input: 3xN matrix representing the landmarks in front of the camera
                  const Eigen::MatrixXd& mMeas,///<Input: 3xN measurements in the normalised plane or unit sphere
                  const int nMaxNumIters,
                  const double dTol,
                  const double dEpsilon,
                  Eigen::Matrix3d& mR,///<Input/Output: initial rotation estimate, this will also contain the estimated rotation
                  Eigen::Vector3d& vt,///Output: estimated translation
                  int& nNumIterations,
                  double& dObjError,
                  bool bUseRtForInitialisation = false );

    
    void objpose_robust
        ( const Eigen::MatrixXd& mP3DNotC, ///<Input: 3xN matrix representing the landmarks in front of the camera
          const Eigen::MatrixXd& mMeas,///<Input: 3xN measurements in the normalised plane or unit sphere
          const int nMaxNumIters,
          const double dTol,
          const double dEpsilon,
          const double dExpectedNoiseStd,
          Eigen::Matrix3d& mR,///<Input/Output: initial rotation estimate, this will also contain the estimated rotation
          Eigen::Vector3d& vt,///<Input/Output: initial translation estimated translation
          int& nNumIterations,
          double& dObjError,
          std::vector<bool>& vInliers,
          bool bUseRForInitialisation = false,
          bool bUsetForInitialisation = false );
}

#endif
