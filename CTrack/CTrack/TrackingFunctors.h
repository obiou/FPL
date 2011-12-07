


///
/// This file contains functors that describe the operations that will be applied for tracking.
/// In particular it defines the warping function, the way the parameters should be updated
/// and the Jacobian computed for each line. An extra function (that can be empty) defines possible regularisers that can be added
/// (that correspond to values added to the Hessian and J'*error.
///
#ifndef TRACKING_FUNCTORS_H

#define TRACKING_FUNCTORS_H

#include <CTrack/Homography.h>

////////////////////////////////////////////////////////////////////////////////
/// Parameters used for regularisation
namespace CTrack {
    const double REG_SIGMA_ALPHA = 0.2;
    const double REG_SIGMA_BETA  = 30;
    double REG_MULT_ERROR( const double dRMS ) { const double dSc = dRMS/10; return dSc*dSc; } 
        //{ return dRMS; } //
}

////////////////////////////////////////////////////////////////////////////////
namespace CTrack {

    ////////////////////////////////////////////////////////////////////////////
    void AddAffineIlluminationRegulariser
        ( const int DOF, ///< Input: number of parameters being estimated
          const double dNumPixels, ///<Input: Number of pixels used in the minimisation
          double* JtJ, ///<Input/Output: Hessian (J'*J) in upper triangular form
          double* JtE, ///<Input/Output: J'*error
          CTrack::TrackingResults* pTrackingResults, ///<Input: provides dAlpha, dAlphaPrev, dBeta, dBetaPrev
          double* dRMS ///<Input/Output: RMS from the dNumPixels reprojections
          )
    {
        const double dAlpha     = pTrackingResults->GetAlpha();
        const double dAlphaPrev = pTrackingResults->GetAlphaPrev();
        const double dBeta      = pTrackingResults->GetBeta();
        const double dBetaPrev  = pTrackingResults->GetBetaPrev();

        const int DOF_T    = DOF*(DOF+1)/2;

        const double dFact = REG_MULT_ERROR( *dRMS )*sqrt( dNumPixels );
        double dSumSqError =  (*dRMS)*(*dRMS)*dNumPixels;

        // Add regulariser on alpha
        const double dFactAlpha = dFact/REG_SIGMA_ALPHA;
        const double dJRegAlpha = dFactAlpha;
        //const double dRegErrorAlpha = dFactAlpha*(dAlpha-1);
        const double dRegErrorAlpha = dFactAlpha*(dAlpha-dAlphaPrev);
    
        JtJ[ DOF_T - DOF - 1 ] += dJRegAlpha*dJRegAlpha;
        JtE[ DOF - 2 ]   += dJRegAlpha*dRegErrorAlpha;
    
        dSumSqError += dRegErrorAlpha*dRegErrorAlpha;
    
        // Add regulariser on beta
        const double dFactBeta  = dFact/REG_SIGMA_BETA;
        const double dJRegBeta  = dFactBeta;
        //const double dRegErrorBeta  = dFactBeta*dBeta;
        const double dRegErrorBeta  = dFactBeta*(dBeta-dBetaPrev);
    
        JtJ[ DOF_T - 1 ] += dJRegBeta*dJRegBeta;
        JtE[ DOF - 1 ]   += dJRegBeta*dRegErrorBeta;
    
        dSumSqError += dRegErrorBeta*dRegErrorBeta;
    
        // Update RMS error
        *dRMS = sqrt( dSumSqError/(dNumPixels+2) );
    } 
 
    ////////////////////////////////////////////////////////////////////////////////
    class TranslationFunctor
    {
    public:
        typedef LineJacobianTranslation LJ;

        const static TrackingMotionModel MOTION_MODEL = TRANSLATION;
        const static TrackingIlluminationModel ILLUMINATION_MODEL = NONE;

        const static bool BLUR = false;

        static bool UpdateResult( CTrack::TrackingResults* pTrackingResults,
                                  double* JtJ, double* JtE, 
                                  double* dNormUpdate )
        {
            Homography HEst = pTrackingResults->GetHomography();
            bool bSuccess = HEst.R2Update( JtJ, JtE, dNormUpdate );
            if( bSuccess ) { 
                pTrackingResults->SetHomography( HEst ); 
            }
            return bSuccess;
        }

        static void Regularise( int, int, double*, double*, CTrack::TrackingResults*, double* ) {}
    };

    ////////////////////////////////////////////////////////////////////////////////
    class TranslationIlluminationFunctor
    {
    public:
        typedef LineJacobianTranslationIllumination LJ;

        const static TrackingMotionModel MOTION_MODEL = TRANSLATION;
        const static TrackingIlluminationModel ILLUMINATION_MODEL = AFFINE_ILLUM;

        const static bool BLUR = false;

        static bool UpdateResult( CTrack::TrackingResults* pTrackingResults,
                                  double* JtJ, double* JtE, 
                                  double* dNormUpdate )
        {
            Homography HEst = pTrackingResults->GetHomography();
            bool bSuccess = HEst.R2Update( JtJ, JtE, dNormUpdate );
            if( bSuccess ) { 
                pTrackingResults->SetHomography( HEst ); 
            }
            return bSuccess;
        }

        static void Regularise
            ( const int DOF, ///< Input: number of parameters being estimated
              const double dNumPixels, ///<Input: Number of pixels used in the minimisation
              double* JtJ, ///<Input/Output: Hessian (J'*J) in upper triangular form
              double* JtE, ///<Input/Output: J'*error
              CTrack::TrackingResults* pTrackingResults, ///<Input: provides dAlpha, dAlphaPrev, dBeta, dBetaPrev
              double* dRMS ///<Input/Output: RMS from the dNumPixels reprojections
              ) {
            AddAffineIlluminationRegulariser( DOF, dNumPixels, JtJ, JtE, pTrackingResults, dRMS );
        }
    };

    ////////////////////////////////////////////////////////////////////////////////
    class SE2Functor
    {
    public:
        typedef LineJacobianSE2 LJ;

        const static TrackingMotionModel MOTION_MODEL = SE2;
        const static TrackingIlluminationModel ILLUMINATION_MODEL = NONE;


        const static bool BLUR = false;

        static bool UpdateResult( CTrack::TrackingResults* pTrackingResults,
                                  double* JtJ, double* JtE, 
                                  double* dNormUpdate )
        {
            Homography HEst = pTrackingResults->GetHomography();
            bool bSuccess = HEst.SE2Update( JtJ, JtE, dNormUpdate );
            if( bSuccess ) { 
                pTrackingResults->SetHomography( HEst ); 
            }
            return bSuccess;
        }

        static void Regularise( int, int, double*, double*, CTrack::TrackingResults*, double* ) {}
    };

    ////////////////////////////////////////////////////////////////////////////////
    class SE2IlluminationFunctor
    {
    public:
        typedef LineJacobianSE2Illumination LJ;

        const static TrackingMotionModel MOTION_MODEL = SE2;
        const static TrackingIlluminationModel ILLUMINATION_MODEL = AFFINE_ILLUM;


        const static bool BLUR = false;

        static bool UpdateResult( CTrack::TrackingResults* pTrackingResults,
                                  double* JtJ, double* JtE, 
                                  double* dNormUpdate )
        {
            Homography HEst = pTrackingResults->GetHomography();
            bool bSuccess = HEst.SE2Update( JtJ, JtE, dNormUpdate );
            if( bSuccess ) { 
                pTrackingResults->SetHomography( HEst ); 
            }
            return bSuccess;
        }

        static void Regularise
            ( const int DOF, ///< Input: number of parameters being estimated
              const double dNumPixels, ///<Input: Number of pixels used in the minimisation
              double* JtJ, ///<Input/Output: Hessian (J'*J) in upper triangular form
              double* JtE, ///<Input/Output: J'*error
              CTrack::TrackingResults* pTrackingResults, ///<Input: provides dAlpha, dAlphaPrev, dBeta, dBetaPrev
              double* dRMS ///<Input/Output: RMS from the dNumPixels reprojections
              ) {
            AddAffineIlluminationRegulariser( DOF, dNumPixels, JtJ, JtE, pTrackingResults, dRMS );
        }
    };

    ////////////////////////////////////////////////////////////////////////////////
    class AffineFunctor
    {
    public:
        typedef LineJacobianAffine LJ;

        const static TrackingMotionModel MOTION_MODEL = AFFINE;
        const static TrackingIlluminationModel ILLUMINATION_MODEL = NONE;

        const static bool BLUR = false;

        static bool UpdateResult( CTrack::TrackingResults* pTrackingResults,
                                  double* JtJ, double* JtE, 
                                  double* dNormUpdate )
        { 
            Homography HEst = pTrackingResults->GetHomography();
            bool bSuccess = HEst.SL2R2Update( JtJ, JtE, dNormUpdate );
            if( bSuccess ) { 
                pTrackingResults->SetHomography( HEst ); 
            }
            return bSuccess;
        }

        static void Regularise( int, int, double*, double*, CTrack::TrackingResults*, double* ) {}
    };

    ////////////////////////////////////////////////////////////////////////////////
    class AffineIlluminationFunctor
    {
    public:
        typedef LineJacobianAffineIllumination LJ;

        const static TrackingMotionModel MOTION_MODEL = AFFINE;
        const static TrackingIlluminationModel ILLUMINATION_MODEL = AFFINE_ILLUM;

        const static bool BLUR = false;

        static bool UpdateResult( CTrack::TrackingResults* pTrackingResults,
                                  double* JtJ, double* JtE, 
                                  double* dNormUpdate )
        { 
            Homography HEst = pTrackingResults->GetHomography();
            double dAlpha   = pTrackingResults->GetAlpha();
            double dBeta    = pTrackingResults->GetBeta();

            bool bSuccess = 
                HEst.SL2R2IllumUpdate( JtJ, JtE, dNormUpdate, &dAlpha, &dBeta );
            if( bSuccess ) { 
                pTrackingResults->SetHomography( HEst ); 
                pTrackingResults->SetAlpha( dAlpha ); 
                pTrackingResults->SetBeta( dBeta ); 
            }
            return bSuccess;
        }

        static void Regularise
            ( const int DOF, ///< Input: number of parameters being estimated
              const double dNumPixels, ///<Input: Number of pixels used in the minimisation
              double* JtJ, ///<Input/Output: Hessian (J'*J) in upper triangular form
              double* JtE, ///<Input/Output: J'*error
              CTrack::TrackingResults* pTrackingResults, ///<Input: provides dAlpha, dAlphaPrev, dBeta, dBetaPrev
              double* dRMS ///<Input/Output: RMS from the dNumPixels reprojections
              ) {
            AddAffineIlluminationRegulariser( DOF, dNumPixels, JtJ, JtE, pTrackingResults, dRMS );
        }
    };

    ////////////////////////////////////////////////////////////////////////////////
    class HomographyFunctor
    {
    public:
        typedef LineJacobianHomography LJ;

        const static TrackingMotionModel MOTION_MODEL = HOMOGRAPHY;
        const static TrackingIlluminationModel ILLUMINATION_MODEL = NONE;

        const static bool BLUR = false;

        static bool UpdateResult( CTrack::TrackingResults* pTrackingResults,
                                  double* JtJ, double* JtE, 
                                  double* dNormUpdate )
        { 
            Homography HEst = pTrackingResults->GetHomography();
            bool bSuccess = HEst.SL3Update( JtJ, JtE, dNormUpdate );
            if( bSuccess ) { 
                pTrackingResults->SetHomography( HEst ); 
            }
            return bSuccess;
        }

        static void Regularise( int, int, double*, double*, CTrack::TrackingResults*, double* ) {}
    };

    ////////////////////////////////////////////////////////////////////////////////
    class HomographyIlluminationFunctor
    {
    public: 
        typedef LineJacobianHomographyIllumination LJ;

        const static TrackingMotionModel MOTION_MODEL = HOMOGRAPHY;
        const static TrackingIlluminationModel ILLUMINATION_MODEL = AFFINE_ILLUM;

        const static bool BLUR = false;

        static bool UpdateResult( CTrack::TrackingResults* pTrackingResults,
                                  double* JtJ, double* JtE, 
                                  double* dNormUpdate )
        { 
            Homography HEst = pTrackingResults->GetHomography();
            double dAlpha   = pTrackingResults->GetAlpha();
            double dBeta    = pTrackingResults->GetBeta();

            bool bSuccess = 
                HEst.SL3IllumUpdate( JtJ, JtE, dNormUpdate, &dAlpha, &dBeta );
            if( bSuccess ) { 
                pTrackingResults->SetHomography( HEst ); 
                pTrackingResults->SetAlpha( dAlpha ); 
                pTrackingResults->SetBeta( dBeta ); 
            }
            return bSuccess;
        }

        static void Regularise
            ( const int DOF, ///< Input: number of parameters being estimated
              const double dNumPixels, ///<Input: Number of pixels used in the minimisation
              double* JtJ, ///<Input/Output: Hessian (J'*J) in upper triangular form
              double* JtE, ///<Input/Output: J'*error
              CTrack::TrackingResults* pTrackingResults, ///<Input: provides dAlpha, dAlphaPrev, dBeta, dBetaPrev
              double* dRMS ///<Input/Output: RMS from the dNumPixels reprojections
              ) {
            AddAffineIlluminationRegulariser( DOF, dNumPixels, JtJ, JtE, pTrackingResults, dRMS );
        }
    };

    ////////////////////////////////////////////////////////////////////////////////
    class HomographyBlurMagnitudeFunctor
    {
    public:        
        typedef LineJacobianHomographyBlurMagn LJ;

        const static TrackingMotionModel MOTION_MODEL = HOMOGRAPHY;
        const static TrackingIlluminationModel ILLUMINATION_MODEL = NONE;

        const static bool BLUR = true;

        static bool UpdateResult( CTrack::TrackingResults* pTrackingResults,
                                  double* JtJ, double* JtE, 
                                  double* dNormUpdate )
        { 
            Homography HEst = pTrackingResults->GetHomography();
            double dLambda  = pTrackingResults->GetBlurMagn();
            bool bSuccess = HEst.SL3BlurMagnUpdate( JtJ, JtE, dNormUpdate, &dLambda );
            if( bSuccess ) { 
                pTrackingResults->SetHomography( HEst ); 
                pTrackingResults->SetBlurMagn( dLambda ); 
            }
            return bSuccess;
        }

        static void Regularise( int, int, double*, double*, CTrack::TrackingResults*, double* ) {}
    };

    ////////////////////////////////////////////////////////////////////////////////
    class HomographyBlurMagnitudeIlluminationFunctor
    {
    public:
        typedef LineJacobianHomographyBlurMagnIllumination LJ;

        const static TrackingMotionModel MOTION_MODEL = HOMOGRAPHY;
        const static TrackingIlluminationModel ILLUMINATION_MODEL = AFFINE_ILLUM;

        const static bool BLUR = true;

        static bool UpdateResult( CTrack::TrackingResults* pTrackingResults,
                                  double* JtJ, double* JtE, 
                                  double* dNormUpdate )
        { 
            Homography HEst = pTrackingResults->GetHomography();
            double dLambda  = pTrackingResults->GetBlurMagn();
            double dAlpha   = pTrackingResults->GetAlpha();
            double dBeta    = pTrackingResults->GetBeta();
            bool bSuccess = 
                HEst.SL3BlurMagnIllumUpdate( JtJ, JtE, dNormUpdate,
                                             &dLambda, &dAlpha, &dBeta );
        
            if( bSuccess ) { 
                pTrackingResults->SetHomography( HEst );
                pTrackingResults->SetBlurMagn( dLambda );
                pTrackingResults->SetAlpha( dAlpha );
                pTrackingResults->SetBeta( dBeta );
            }  
            return bSuccess;
        }

        static void Regularise
            ( const int DOF, ///< Input: number of parameters being estimated
              const double dNumPixels, ///<Input: Number of pixels used in the minimisation
              double* JtJ, ///<Input/Output: Hessian (J'*J) in upper triangular form
              double* JtE, ///<Input/Output: J'*error
              CTrack::TrackingResults* pTrackingResults, ///<Input: provides dAlpha, dAlphaPrev, dBeta, dBetaPrev
              double* dRMS ///<Input/Output: RMS from the dNumPixels reprojections
              ) {
            AddAffineIlluminationRegulariser( DOF, dNumPixels, JtJ, JtE, pTrackingResults, dRMS );
        }
    };
}

#endif
