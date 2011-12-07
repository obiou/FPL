//#include <CTrack/Homography.h>
#include "Homography.h"

#include <stdio.h>

#include <eigen3/Eigen/Cholesky>
#include <eigen3/Eigen/LU>

#include <CTrack/Jacobians.h>
#include <CTrack/MatrixLogarithm.h>

namespace CTrack {
    ////////////////////////////////////////////////////////////////////////////
    bool SolveNormalEquations( const int nDOF, double* JtJ, double* JtE,
                               Eigen::VectorXd& vESMUpdate ) {
        Eigen::MatrixXd mJtJ = MakeFullSymmetricMatrix( nDOF, JtJ );
        //std::cout << "mJtJ: " << mJtJ << std::endl;
        Eigen::Map<Eigen::VectorXd> vJtE( JtE, nDOF );
        //std::cout << "mJtE: " << vJtE << std::endl;
        vJtE = -vJtE;
        //bool bSuccess = 
        //    mJtJ.ldlt().solve( vJtE, &vESMUpdate ); 
        bool bSuccess = true;
        vESMUpdate = mJtJ.ldlt().solve( vJtE );

        vJtE = -vJtE;
        return bSuccess;
    }

    ////////////////////////////////////////////////////////////////////////////
    Eigen::MatrixXd MakeFullSymmetricMatrix( const int nDOF, double* JtJ ) {
        Eigen::MatrixXd mJtJ( nDOF, nDOF );
        double* pJtJ = NULL;
        // Fill the lower triangular part
        pJtJ = JtJ;
        for( int ii=0; ii<nDOF; ii++ ) {
            for( int jj=0; jj<=ii; jj++ ) {
                mJtJ( ii, jj ) = *pJtJ++;
            }
        }
        // Fill the upper triangular part
        pJtJ = JtJ;
        for( int jj=0; jj<nDOF; jj++ ) {
            for( int ii=0; ii<=jj; ii++ ) {
                mJtJ( ii, jj ) = *pJtJ++;
            }
        }
        return mJtJ;
    }

    ////////////////////////////////////////////////////////////////////////////
    Eigen::Matrix3d expm( const Eigen::Matrix3d& mM ) {
        // Scale the matrix by 2^n for better numerical stability
        const double dScale = log2( mM.lpNorm<Eigen::Infinity>() ); // divide by two as we later take the sqrt
        int nPow = std::max(0,(int)ceil(dScale));
        Eigen::Matrix3d mExpM = expm_series( mM/(1<<nPow) );
        for( int ii=0; ii<nPow; ii++ ){
            mExpM = mExpM*mExpM;
        }
        return mExpM;
    }

    ////////////////////////////////////////////////////////////////////////////
    Eigen::Matrix3d ComputeSL3Update( const Eigen::VectorXd& vESMUpdate ) {
        Eigen::VectorXd vFullESMUpdate( DOF_HOMOG );
        for( int ii=0; ii<vESMUpdate.size(); ii++ ) {
            vFullESMUpdate[ii] = vESMUpdate[ii];
        }
        for( int ii=vESMUpdate.size(); ii<DOF_HOMOG; ii++ ) {
            vFullESMUpdate[ii] = 0;
        }
        Eigen::Matrix3d mGUpdate = MakeSumGenerators( vFullESMUpdate );
        return expm( mGUpdate );
    }

    ////////////////////////////////////////////////////////////////////////////
    Eigen::Matrix3d MakeSumGenerators( const Eigen::VectorXd& vESMUpdate ) {
        //std::cout << "vESMUpdate: " << std::endl << vESMUpdate << std::endl;
        Eigen::Matrix3d mGUpdate;
        mGUpdate << 
            vESMUpdate[3]/3+vESMUpdate[4], vESMUpdate[2]+vESMUpdate[5], vESMUpdate[0], 
            -vESMUpdate[2], vESMUpdate[3]/3-vESMUpdate[4], vESMUpdate[1], 
            vESMUpdate[6], vESMUpdate[7], -2*vESMUpdate[3]/3;
        //std::cout << "mGUpdate: " << std::endl << mGUpdate << std::endl;
        return mGUpdate;
    }

    ////////////////////////////////////////////////////////////////////////////
    CTrack::Homography GenerateScaled( double dScale ) {
        CTrack::Homography HScaled;
        HScaled.Set( 0, 0, dScale );
        HScaled.Set( 1, 1, dScale );
#if 0
        HScaled.Set( 0, 2, 0.5 );
        HScaled.Set( 1, 2, 0.5 );
#endif
        return HScaled;
    }

    ////////////////////////////////////////////////////////////////////////////
    Eigen::Matrix3d expm_series( const Eigen::Matrix3d& mM ) {
        Eigen::Matrix3d mExpM = Eigen::Matrix3d::Identity();
        Eigen::Matrix3d mTerm = mM;
        const double dTol = 1e-10;
        const double dMaxIter = 100.;
        for( double ii=2.; ii < dMaxIter; ii++ ) {
            mExpM += mTerm;
            mTerm = (mTerm*mM)/ii;
            if( mTerm.lpNorm<Eigen::Infinity>() < dTol ) {
                mExpM += mTerm;
                break;
            }
        }
        return mExpM;
    }

    ////////////////////////////////////////////////////////////////////////////
    float randf() {
        return (float)rand()/RAND_MAX;
    }
    
    ////////////////////////////////////////////////////////////////////////////
    void rand_gaussf( float fMean, float fStd, float& y1, float& y2 ) {
        rand_gaussf( y1, y2 );
        y1 = fMean + y1*fStd;
        y2 = fMean + y2*fStd;
    }

    ////////////////////////////////////////////////////////////////////////////
    void rand_gaussf( float& y1, float& y2 ) {
        float x1, x2, w;
        do {
            x1 = 2.0 * randf() - 1.0;
            x2 = 2.0 * randf() - 1.0;
            w = x1 * x1 + x2 * x2;
        } while ( w >= 1.0 );
        
        w = sqrt( (-2.0 * log( w ) ) / w );
        y1 = x1 * w;
        y2 = x2 * w;        
    }

    ////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////
    void Homography::inv() {
        m_mH = m_mH.inverse().eval();
    }

    ////////////////////////////////////////////////////////////////////////
    void Homography::logm() {
        m_mH = CTrack::logm( m_mH );
    }

    ////////////////////////////////////////////////////////////////////////
    void Homography::SL3Update( int nDOF, double* dUpdate ) {
        Eigen::Map<Eigen::VectorXd> vESMUpdate( dUpdate, nDOF );
        m_mH *= ComputeSL3Update( vESMUpdate );
    }

    ////////////////////////////////////////////////////////////////////////
    bool Homography::SL2R2Update( double* JtJ,        ///< Input
                      double* JtE,        ///< Input
                      double* dNormUpdate ///< Output
                      ) {
        return _GenericSL3Update( DOF_AFFINE, JtJ, JtE, dNormUpdate );
    }

    ////////////////////////////////////////////////////////////////////////
    bool Homography::SE2Update( double* JtJ,        ///< Input
                    double* JtE,        ///< Input
                    double* dNormUpdate ///< Output
                    ) {
        return _GenericSL3Update( DOF_SE2, JtJ, JtE, dNormUpdate );
    }

    ////////////////////////////////////////////////////////////////////////
    bool Homography::SL3Update( double* JtJ,        ///< Input
                    double* JtE,        ///< Input
                    double* dNormUpdate ///< Output
                    ) {
        return _GenericSL3Update( DOF_HOMOG, JtJ, JtE, dNormUpdate );
    }

    ////////////////////////////////////////////////////////////////////////
    bool Homography::SL2R2IllumUpdate( double* JtJ,        ///< Input
                           double* JtE,        ///< Input
                           double* dNormUpdate, ///< Output
                           double* dAlpha, ///< Output
                           double* dBeta   ///< Output
                           ) {
        Eigen::VectorXd vESMUpdate;
        bool bSuccess = 
            SolveNormalEquations( DOF_AFFINE_ILLUM, JtJ, JtE, vESMUpdate );
 
        if( !bSuccess ) {
            return bSuccess;
        }

        *dNormUpdate = vESMUpdate.norm();
           
        *dAlpha += vESMUpdate[ DOF_AFFINE ];
        *dBeta  += vESMUpdate[ DOF_AFFINE + 1 ]; // last coeff
        //m_mH *= ComputeSL3Update( vESMUpdate.start( DOF_AFFINE ) ); // first 6 coeffs
        m_mH *= ComputeSL3Update( vESMUpdate.head( DOF_AFFINE ) ); // first 6 coeffs
        return bSuccess;
    }

    ////////////////////////////////////////////////////////////////////////
    bool Homography::SL3IllumUpdate( double* JtJ,        ///< Input
                         double* JtE,        ///< Input
                         double* dNormUpdate, ///< Output
                         double* dAlpha, ///< Output
                         double* dBeta   ///< Output
                         ) {
        Eigen::VectorXd vESMUpdate;
        bool bSuccess = 
            SolveNormalEquations( DOF_HOMOG_ILLUM, JtJ, JtE, vESMUpdate );
 
        if( !bSuccess ) {
            return bSuccess;
        }

        *dNormUpdate = vESMUpdate.norm();
           
        *dAlpha += vESMUpdate[ DOF_HOMOG ];
        *dBeta  += vESMUpdate[ DOF_HOMOG+1 ]; // last coeff
        //m_mH *= ComputeSL3Update( vESMUpdate.start( DOF_HOMOG ) ); // first 8 coeffs
        m_mH *= ComputeSL3Update( vESMUpdate.head( DOF_HOMOG ) ); // first 8 coeffs
        return bSuccess;
    }

    ////////////////////////////////////////////////////////////////////////
    bool Homography::SL3BlurMagnUpdate( double* JtJ,        ///< Input
                            double* JtE,        ///< Input
                            double* dNormUpdate, ///< Output
                            double* dLambda
                            ) {
        Eigen::VectorXd vESMUpdate;
        bool bSuccess = 
            SolveNormalEquations( DOF_HOMOG_BLUR_MAGN, JtJ, JtE, vESMUpdate );
 
        if( !bSuccess ) {
            return bSuccess;
        }

        *dNormUpdate = vESMUpdate.norm();

        if( *dLambda + vESMUpdate[ DOF_HOMOG ] < 0 ||
            *dLambda + vESMUpdate[ DOF_HOMOG ] > 1 ) {
            // In the case where vESMUpdate is dodgy,
            // recompute the update without evaluating lambda
            double JtJ_nl[ DOF_T( DOF_HOMOG ) ];
            double JtE_nl[ DOF_HOMOG ];
            double* tmp_JtJ_nl = JtJ_nl;
            double* tmp_JtE_nl = JtE_nl;

            // Copy values except for the last row and column
            for( int nCol=0; nCol<DOF_HOMOG; nCol++ ) {
                for( int nRow=nCol; nRow<DOF_HOMOG; nRow++ ) {
                    *tmp_JtJ_nl++ = *JtJ++;
                }
                *tmp_JtE_nl++ = *JtE++;
            }
            std::cout << "Computing update without the magnitude (lambda: " 
                      << *dLambda + vESMUpdate[ DOF_HOMOG ] << ")"
                      << std::endl;
            return SL3Update( JtJ_nl, JtE_nl, dNormUpdate );
        }
        else {
            *dLambda += vESMUpdate[ DOF_HOMOG ]; // last coeff
            //m_mH *= ComputeSL3Update( vESMUpdate.start( DOF_HOMOG ) ); // first 8 coeffs
            m_mH *= ComputeSL3Update( vESMUpdate.head( DOF_HOMOG ) ); // first 8 coeffs
        }
        return bSuccess;
    }

    ////////////////////////////////////////////////////////////////////////
    bool Homography::SL3BlurMagnIllumUpdate( double* JtJ,        ///< Input
                                 double* JtE,        ///< Input
                                 double* dNormUpdate, ///< Output
                                 double* dLambda,
                                 double* dAlpha,
                                 double* dBeta
                                 ) {
        Eigen::VectorXd vESMUpdate;
        bool bSuccess = 
            SolveNormalEquations( DOF_HOMOG_ILLUM_BLUR_MAGN, JtJ, JtE, vESMUpdate );
 
        if( !bSuccess ) {
            return bSuccess;
        }

        *dNormUpdate = vESMUpdate.norm();

        if( *dLambda + vESMUpdate[ DOF_HOMOG ] < 0 ||
            *dLambda + vESMUpdate[ DOF_HOMOG ] > 1 ) {
            *dAlpha  += vESMUpdate[ DOF_HOMOG + 1 ];
            *dBeta   += vESMUpdate[ DOF_HOMOG + 2 ]; // last coeff
            //m_mH *= ComputeSL3Update( vESMUpdate.start( DOF_HOMOG ) ); // first 8 coeffs
            m_mH *= ComputeSL3Update( vESMUpdate.head( DOF_HOMOG ) ); // first 8 coeffs
            return bSuccess;
        }

        if( 0 ) {// *dLambda + vESMUpdate[ DOF_HOMOG ] < 0 ||
            //*dLambda + vESMUpdate[ DOF_HOMOG ] > 1 ) {
            // In the case where vESMUpdate is dodgy,
            // add dampening to the Hessian
            double mu = 1;
            int nMaxIter = 20;
            int nNumIter = 0;
            while( *dLambda + vESMUpdate[ DOF_HOMOG ] < 0 ||
                   *dLambda + vESMUpdate[ DOF_HOMOG ] > 1 ) {
                printf( "." ); // for debugging
                mu *= 2;
                double* pJtJ = JtJ;
                for( int ii=0; ii<DOF_HOMOG_ILLUM_BLUR_MAGN; ii++ ) {
                    for( int jj=0; jj<=ii; jj++ ) {
                        if( jj == ii && jj == DOF_HOMOG ) { // lambda part 
                            *pJtJ += mu;
                        }
                        pJtJ++;
                    }
                }
                bSuccess = 
                    SolveNormalEquations( DOF_HOMOG_ILLUM_BLUR_MAGN, JtJ, JtE, vESMUpdate );
                if( !bSuccess ) {
                    return bSuccess;
                }
                nNumIter++;
                if( nNumIter > nMaxIter ) {
                    printf( "/" );
                    // Update all variables except lambda.
                    *dAlpha  += vESMUpdate[ DOF_HOMOG + 1 ];
                    *dBeta   += vESMUpdate[ DOF_HOMOG + 2 ]; // last coeff
                    //m_mH *= ComputeSL3Update( vESMUpdate.start( DOF_HOMOG ) ); // first 8 coeffs
                    m_mH *= ComputeSL3Update( vESMUpdate.head( DOF_HOMOG ) ); // first 8 coeffs
                    return bSuccess;
                }
            }

        }

        *dLambda += vESMUpdate[ DOF_HOMOG ];
        *dAlpha  += vESMUpdate[ DOF_HOMOG + 1 ];
        *dBeta   += vESMUpdate[ DOF_HOMOG + 2 ]; // last coeff
        //m_mH *= ComputeSL3Update( vESMUpdate.start( DOF_HOMOG ) ); // first 8 coeffs
        m_mH *= ComputeSL3Update( vESMUpdate.head( DOF_HOMOG ) ); // first 8 coeffs
           
        return bSuccess;
    }
}
