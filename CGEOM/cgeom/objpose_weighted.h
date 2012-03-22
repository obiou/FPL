#ifndef OBJPOSE_WEIGHTED_H
#define OBJPOSE_WEIGHTED_H

#include <eigen3/Eigen/Core>

#include <vector>

#define PRINT_MATRIX( M ) std::cout << #M << ": " << std::endl << M << std::endl;
#define PRINT_SCALAR( M ) std::cout << #M << ": " << M << std::endl;

namespace CGEOM {
    ////////////////////////////////////////////////////////////////////////////
    /// Weights are set at 1. Should produce the same
    /// output as standard objpose.
    class OneWeights {
    public:
        Eigen::VectorXd operator()( const double,
                                    const double,                     
                                    const Eigen::MatrixXd& mQ, ///<Input
                                    const std::vector<Eigen::Matrix3d>&, ///<Input
                                    std::vector<bool>& ///<Input/Output
                                    ) const;
    };

    ////////////////////////////////////////////////////////////////////////////
    /// Basic robust cost function *without* inverse depths weighting
    class TukeyWeights {
    public:
        Eigen::VectorXd operator()( const double dExpectedNoiseStd,
                                    const double dError,                     
                                    const Eigen::MatrixXd& mQ, ///<Input
                                    const std::vector<Eigen::Matrix3d>& vV, ///<Input
                                    std::vector<bool>& vInliers ///<Input/Output
                                    ) const;
    };

    ////////////////////////////////////////////////////////////////////////////
    /// Inverse depth weighting (to avoid scene depth bias)
    class InvSqDistWeights {
    public:
        Eigen::VectorXd operator()( const double dExpectedNoiseStd,
                                    const double dError,                     
                                    const Eigen::MatrixXd& mQ, ///<Input
                                    const std::vector<Eigen::Matrix3d>& vV, ///<Input
                                    std::vector<bool>& vInliers ///<Input/Output
                                    ) const;
    };

    ////////////////////////////////////////////////////////////////////////////
    /// Robust cost function combined *with* inverse depth weighting
    class TukeyInvSqDistWeights {
    public:
        Eigen::VectorXd operator()( const double dExpectedNoiseStd,
                                    const double dError,                     
                                    const Eigen::MatrixXd& mQ, ///<Input
                                    const std::vector<Eigen::Matrix3d>& vV, ///<Input
                                    std::vector<bool>& vInliers ///<Input/Output
                                    ) const;
    };

    ////////////////////////////////////////////////////////////////////////////
    template<class WeightFunc>
    void objpose_weighted
        ( const Eigen::MatrixXd& mP3DNotC, ///<Input: 3xN matrix representing the landmarks in front of the camera
          const Eigen::MatrixXd& mMeas,///<Input: 3xN measurements in the normalised plane or unit sphere
          const int nMaxNumIters,
          const double dTol,
          const double dEpsilon,
          const double dExpectedNoiseStd,
          const WeightFunc& weight_func,
          Eigen::Matrix3d& mR,///<Input/Output: initial rotation estimate, this will also contain the estimated rotation
          Eigen::Vector3d& vt,///<Input/Output: initial translation estimated translation
          int& nNumIterations,
          double& dObjError,
          std::vector<bool>& vInliers,
          bool bUseRForInitialisation = false,
          bool bUsetForInitialisation = false );

    ////////////////////////////////////////////////////////////////////////////
    template<class WeightFunc>
    void objpose_weighted
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
          bool bUsetForInitialisation = false ) {
        CGEOM::objpose_weighted<WeightFunc>
            ( mP3DNotC, mMeas, nMaxNumIters,
              dTol, dEpsilon, dExpectedNoiseStd,
              WeightFunc(),
              mR, vt, nNumIterations,
              dObjError, vInliers, bUseRForInitialisation,
              bUsetForInitialisation
              );
    }
}

#endif
