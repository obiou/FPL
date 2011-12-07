// Copyright (C) 2010 Christopher Mei (christopher.m.mei@gmail.com)
//
// This file is part of the CTrack Library. This library is free
// software; you can redistribute it and/or modify it under the
// terms of the GNU Lesser General Public License as published by the
// Free Software Foundation; either version 2, or (at your option)
// any later version.

// This library is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU Lesser General Public License for more details.

// You should have received a copy of the GNU Lesser General Public
// License along with this library; see the file COPYING.LESSER. If
// not, write to the Free Software Foundation, 59 Temple Place - Suite
// 330, Boston, MA 02111-1307, USA.

// As a special exception, you may use this file as part of a free software
// library without restriction.  Specifically, if other files instantiate
// templates or use macros or inline functions from this file, or you compile
// this file and link it with other files to produce an executable, this
// file does not by itself cause the resulting executable to be covered by
// the GNU Lesser General Public License.  This exception does not however
// invalidate any other reasons why the executable file might be covered by
// the GNU Lesser General Public License.

#ifndef CTRACK_TRACKING_UPDATES_H

#define CTRACK_TRACKING_UPDATES_H

///
/// This module contains the inner loop updates for different tracking cases.
/// These are low-level calls and should not be used directly.
///
#include <CTrack/Homography.h>
#include <CTrack/Jacobians.h>
#include <CTrack/Misc.h>
#include <CTrack/TrackingResults.h>
#include <CTrack/Transforms.h>
#include <CTrack/TransformsHighLevel.h>

#include <iostream>

////////////////////////////////////////////////////////////////////////////////
enum TrackingMotionModel {
    TRANSLATION,
    SE2,
    AFFINE,
    HOMOGRAPHY,
    HOMOGRAPHY_BLUR_MAGN
};

////////////////////////////////////////////////////////////////////////////////
enum TrackingIlluminationModel {
    NONE,
    AFFINE_ILLUM,
    AFFINE_ILLUM_REGULARISED
};

////////////////////////////////////////////////////////////////////////////////
namespace CTrack {
   
    ////////////////////////////////////////////////////////////////////////////
    /// Returns true if the model estimates the inter-frame blur
    bool BlurModel( TrackingMotionModel eTrackingMotionModel ) {
        return eTrackingMotionModel == HOMOGRAPHY_BLUR_MAGN;
    }

    ////////////////////////////////////////////////////////////////////////////////
    /// Adds two extra terms to the least squares:
    /// C1 * \| alphaBest - alphaPrev - 1 \|^2 
    /// C2 * \| betaBest - betaPrev \|^2
    void AddAffineIlluminationRegulariser( const int DOF, ///< Input: number of parameters being estimated
                                           const double dNumPixels, ///<Input: Number of pixels used in the minimisation
                                           double* JtJ, ///<Input/Output: Hessian (J'*J) in upper triangular form
                                           double* JtE, ///<Input/Output: J'*error
                                           CTrack::TrackingResults* pTrackingResults, ///<Input: provides dAlpha, dAlphaPrev, dBeta, dBetaPrev
                                           double* RMS ///Input/Output: RMS from the dNumPixels reprojections
                                           );

    ////////////////////////////////////////////////////////////////////////////
    ///
    /// Assumes pRefPatchGradX and pRefPatchGradY contain the gradient of the reference patch pRefPatch
    /// Assumes the memory has been allocated for the current buffers.
    /// Simplified call assuming the tracking without blur estimation.
    template< class TrackingFunctor, class MaskFunctorT >
        bool SSDUpdateSL3Motion( unsigned char* pRefPatch, ///< Input: will be modified if tracking with estimation of blur
                                 float* pRefPatchGradX,///< Input:
                                 float* pRefPatchGradY,///< Input:
                                 const int nPatchWidth,              ///< Input:
                                 const int nPatchHeight,             ///< Input:
                                 const int nPatchWidthStep,
                                 const unsigned char* pCurImage,     ///< Input:
                                 const int nImageWidth,              ///< Input:
                                 const int nImageHeight,             ///< Input:
                                 const int nImageWidthStep,
                                 const double dBegX, ///< Input: patch upper left X coordinate
                                 const double dBegY, ///< Input: patch upper left Y coordinate
                                 const bool bRegularise,
                                 unsigned char* pCurPatch,     ///< Output:
                                 float* pCurPatchGradX,///< Output:
                                 float* pCurPatchGradY,///< Output:
                                 CTrack::TrackingResults* pTrackingResults, ///< Output:
                                 double* dRMS,                 ///< Output:
                                 double* dNormUpdate,          ///< Output:
                                 MaskFunctorT& maskFctr
                                 );                               

    ////////////////////////////////////////////////////////////////////////////
    ///
    /// Assumes pRefPatchGradX and pRefPatchGradY contain the gradient of the reference patch pRefPatch
    /// Assumes the memory has been allocated for the current buffers.
    ///
    template< class TrackingFunctor, class MaskFunctorT >
        bool SSDUpdateSL3Motion( unsigned char* pRefPatch, ///< Input: will be modified if tracking with estimation of blur
                                 float* pRefPatchGradX,///< Input:
                                 float* pRefPatchGradY,///< Input:
                                 float* pRefPatchWTGradX,///< Input: only used if estimating blur, will contain weighted blurred image in x
                                 float* pRefPatchWTGradY,///< Input: only used if estimating blur, will contain weighted blurred image in y
                                 const int nPatchWidth,              ///< Input:
                                 const int nPatchHeight,             ///< Input:
                                 const int nPatchWidthStep,
                                 const unsigned char* pRefImage, ///< Input: only used if estimating blur, can be set to NULL otherwise
                                 const unsigned char* pCurImage,     ///< Input:
                                 const int nImageWidth,              ///< Input:
                                 const int nImageHeight,             ///< Input:
                                 const int nImageWidthStep,
                                 const double dBegX, ///< Input: patch upper left X coordinate
                                 const double dBegY, ///< Input: patch upper left Y coordinate
                                 const bool bRegularise,
                                 unsigned char* pCurPatch,     ///< Output:
                                 float* pCurPatchGradX,///< Output:
                                 float* pCurPatchGradY,///< Output:
                                 CTrack::TrackingResults* pTrackingResults, ///< Output:
                                 double* dRMS,                 ///< Output:
                                 double* dNormUpdate,          ///< Output:
                                 MaskFunctorT& maskFctr ///< Input: can be used to keep track of out-of-image or masked points (e.g. outliers)
                                 );
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
template< class TrackingFunctor, class MaskFunctorT >
bool CTrack::SSDUpdateSL3Motion( unsigned char* pRefPatch,
                                 float* pRefPatchGradX,
                                 float* pRefPatchGradY,
                                 const int nPatchWidth,
                                 const int nPatchHeight,
                                 const int nPatchWidthStep,
                                 const unsigned char* pCurImage,
                                 const int nImageWidth,
                                 const int nImageHeight,
                                 const int nImageWidthStep,
                                 const double dBegX,
                                 const double dBegY,
                                 const bool bRegularise,
                                 unsigned char* pCurPatch,
                                 float* pCurPatchGradX,
                                 float* pCurPatchGradY,
                                 CTrack::TrackingResults* pTrackingResults,
                                 double* dRMS,
                                 double* dNormUpdate,
                                 MaskFunctorT& maskFctr
                                 )
{
    return SSDUpdateSL3Motion<>( pRefPatch, pRefPatchGradX, pRefPatchGradY,
                                 NULL, NULL, // extra buffers not needed 
                                 nPatchWidth, nPatchHeight, nPatchWidthStep,
                                 NULL, pCurImage, nImageWidth, nImageHeight,  
                                 nImageWidthStep, dBegX, dBegY, bRegularise,
                                 pCurPatch, pCurPatchGradX, pCurPatchGradY,
                                 pTrackingResults,
                                 dRMS, dNormUpdate,     
                                 maskFctr
                                 );
}

////////////////////////////////////////////////////////////////////////////////
// This code is common to all tracking methods that estimate blur
// and assume the current image is blurred and wish to blur the reference
// view to enable accurate tracking.
template< class MaskFunctorT >
void ComputedBlurredReferencImageAndJacobian(  unsigned char* pRefPatch,
                                               float* pRefPatchGradX,
                                               float* pRefPatchGradY,
                                               float* pRefPatchWTGradX,
                                               float* pRefPatchWTGradY,
                                               const int nPatchWidth,
                                               const int nPatchHeight,
                                               const int nPatchWidthStep,
                                               const unsigned char* pRefImage,
                                               const int nImageWidth,
                                               const int nImageHeight,
                                               const int nImageWidthStep,
                                               const double dBegX,
                                               const double dBegY,
                                               CTrack::TrackingResults* pTrackingResults,
                                               MaskFunctorT& maskFctr,
                                               const double dBlurPSFStepInPixels = 1.
                                               ///< Input: defines the sampling of pixels along the PSF to compute the blur
                                               )
{
    // Create weighted blurred template and compute the image Jacobian
    // We use the reference buffer here for convenience, it MUST be overwritten
    // afterwards.               
    // Matlab:
    // JBlur = blur_warping( IRef, HRef, lambdaEst*M, eye(3), nWidth, nHeight, 1 );
    // [Jcol,Jrow] = gradient( JBlur );
    // C++:
    CTrack::Homography HRef;
    HRef.Set( 0, 2, dBegX );
    HRef.Set( 1, 2, dBegY );

    // Compute the inter-frame motion
    CTrack::Homography HPrev = pTrackingResults->GetPrevHomography();
    CTrack::Homography HPrevInv = HPrev; HPrevInv.inv();
    CTrack::Homography M = pTrackingResults->GetHomography();
    M.mult( HPrevInv );
    M.logm();
    M.mult( pTrackingResults->GetBlurMagn() );
    CTrack::Homography Hl = HRef;
    Hl.mult( HPrevInv );

    // This will then be done in the Jacobian call: 
    // JBlur = lambdaEst*Jhomography( Jcol, Jrow, nWidth, nHeight, nNumParametersH );
    CTrack::WarpBlur<MaskFunctorT>( pRefImage, nImageWidth, nImageHeight, nImageWidthStep,
                                    &Hl, &M, &HPrev,
                                    pRefPatch, // we will overwrite this later
                                    nPatchWidth, nPatchHeight, nPatchWidthStep,
                                    maskFctr, dBlurPSFStepInPixels, true // compute blur Jacobian
                                    );
    // This will compute the weighted blur (for the Jacobian)
    CTrack::Gradient( pRefPatch,
                      nPatchWidth, nPatchHeight, nPatchWidthStep,
                      pRefPatchWTGradX, pRefPatchWTGradY );

    // Create blurred reference template and compute the image Jacobian
    // Matlab:
    // [TIRefEst,TIRefMask] = blur_warping( IRef, HRef, lambdaEst*M, eye(3), nWidth, nHeight, 0 );
    // C++: 
    CTrack::WarpBlur<MaskFunctorT>( pRefImage, nImageWidth, nImageHeight, nImageWidthStep,
                                    &Hl, &M, &HPrev,
                                    pRefPatch, // will now be overwritten and contain the right value
                                    nPatchWidth, nPatchHeight, nPatchWidthStep,
                                    maskFctr, dBlurPSFStepInPixels, false  // compute blur (not Jacobian)
                                    );

    CTrack::Gradient( pRefPatch,
                      nPatchWidth, nPatchHeight, nPatchWidthStep,
                      pRefPatchGradX, pRefPatchGradY );

    // The current template will be later estimated by warping from the
    // current image (the one that is assumed blurred)
    // This code is common to all methods.
}

////////////////////////////////////////////////////////////////////////////////
template< class MaskFunctorT >
void WarpNoIllumination(  const unsigned char* pCurImage,
                          const int nImageWidth,
                          const int nImageHeight,
                          const int nImageWidthStep,
                          unsigned char* pCurPatch,
                          const int nPatchWidth,
                          const int nPatchHeight,
                          const int nPatchWidthStep,
                          const double dBegX,
                          const double dBegY,
                          CTrack::TrackingResults* pTrackingResults,
                          MaskFunctorT& maskFctr )
{
    CTrack::Homography HFull;
    HFull.Set( 0, 2, dBegX );
    HFull.Set( 1, 2, dBegY );
    HFull.mult( pTrackingResults->GetHomography() );

    // Test for pure translation (faster)
    if( HFull.IsPureTranslation() ) {
        CTrack::WarpTranslation( pCurImage,
                                 nImageWidth, nImageHeight, nImageWidthStep,
                                 HFull.GetRowMajorPtr()[2],
                                 HFull.GetRowMajorPtr()[5],
                                 pCurPatch,
                                 nPatchWidth, nPatchHeight, nPatchWidthStep,
                                 maskFctr );
    }
    else {
        CTrack::Warp( pCurImage,
                      nImageWidth, nImageHeight, nImageWidthStep,
                      HFull.GetRowMajorPtr(),
                      pCurPatch,
                      nPatchWidth, nPatchHeight, nPatchWidthStep,
                      maskFctr );

    }
}

////////////////////////////////////////////////////////////////////////////////
template< class MaskFunctorT >
void WarpIllumination( const unsigned char* pCurImage,
                       const int nImageWidth,
                       const int nImageHeight,
                       const int nImageWidthStep,
                       unsigned char* pCurPatch,
                       const int nPatchWidth,
                       const int nPatchHeight,
                       const int nPatchWidthStep,
                       const double dBegX,
                       const double dBegY,
                       CTrack::TrackingResults* pTrackingResults,
                       MaskFunctorT& maskFctr )
{
    CTrack::Homography HFull;
    HFull.Set( 0, 2, dBegX );
    HFull.Set( 1, 2, dBegY );
    HFull.mult( pTrackingResults->GetHomography() );

    // Test for pure translation (faster)
    if( HFull.IsPureTranslation() ) {
        CTrack::WarpTranslationIllum( pCurImage,
                                      nImageWidth, nImageHeight, nImageWidthStep,
                                      HFull.GetRowMajorPtr()[2],
                                      HFull.GetRowMajorPtr()[5],
                                      pTrackingResults->GetAlpha(),
                                      pTrackingResults->GetBeta(),
                                      pCurPatch,
                                      nPatchWidth, nPatchHeight, nPatchWidthStep,
                                      maskFctr );
    }
    else {
        CTrack::WarpIllum( pCurImage,
                           nImageWidth, nImageHeight, nImageWidthStep,
                           HFull.GetRowMajorPtr(),
                           pTrackingResults->GetAlpha(),
                           pTrackingResults->GetBeta(),
                           pCurPatch,
                           nPatchWidth, nPatchHeight, nPatchWidthStep,
                           maskFctr );
    }
}

////////////////////////////////////////////////////////////////////////////////
template< class TrackingFunctor, class MaskFunctorT >
bool CTrack::SSDUpdateSL3Motion( unsigned char* pRefPatch,
                                 float* pRefPatchGradX,
                                 float* pRefPatchGradY,
                                 float* pRefPatchWTGradX,
                                 float* pRefPatchWTGradY,
                                 const int nPatchWidth,
                                 const int nPatchHeight,
                                 const int nPatchWidthStep,
                                 const unsigned char* pRefImage,
                                 const unsigned char* pCurImage,
                                 const int nImageWidth,
                                 const int nImageHeight,
                                 const int nImageWidthStep,
                                 const double dBegX,
                                 const double dBegY,
                                 const bool bRegularise,
                                 unsigned char* pCurPatch,
                                 float* pCurPatchGradX,
                                 float* pCurPatchGradY,
                                 CTrack::TrackingResults* pTrackingResults,
                                 double* dRMS,
                                 double* dNormUpdate,
                                 MaskFunctorT& maskFctr )
{
    if( TrackingFunctor::BLUR ) {
        ComputedBlurredReferencImageAndJacobian<MaskFunctorT>( pRefPatch,
                                                               pRefPatchGradX,
                                                               pRefPatchGradY,
                                                               pRefPatchWTGradX,
                                                               pRefPatchWTGradY,
                                                               nPatchWidth,
                                                               nPatchHeight,
                                                               nPatchWidthStep,
                                                               pRefImage,
                                                               nImageWidth,
                                                               nImageHeight,
                                                               nImageWidthStep,
                                                               dBegX, dBegY,
                                                               pTrackingResults,
                                                               maskFctr
                                                               );
    }

    // Could push this code into the functor but it's nice and simple here...
    if( TrackingFunctor::ILLUMINATION_MODEL == NONE ) {
        WarpNoIllumination( pCurImage,
                            nImageWidth, nImageHeight, nImageWidthStep,
                            pCurPatch,
                            nPatchWidth, nPatchHeight, nPatchWidthStep,
                            dBegX, dBegY, pTrackingResults,
                            maskFctr );
    }
    else {
        WarpIllumination( pCurImage,
                          nImageWidth, nImageHeight, nImageWidthStep,
                          pCurPatch,
                          nPatchWidth, nPatchHeight, nPatchWidthStep,
                          dBegX, dBegY, pTrackingResults,
                          maskFctr );
    }

    // 2) Compute gradient of warped patch
    CTrack::Gradient( pCurPatch,
                      nPatchWidth, nPatchHeight, nPatchWidthStep,
                      pCurPatchGradX, pCurPatchGradY );

    // 2') We now set the out-of-image values (that where computed during warping) to nan
    maskFctr.set_to_nan( pCurPatchGradX, nPatchWidth, nPatchHeight );

    bool bSuccess = false;

    double JtJ[ DOF_T( TrackingFunctor::LJ::DOF ) ];
    double JtE[ TrackingFunctor::LJ::DOF ];

    // 3) Compute the Jacobians
    if( TrackingFunctor::BLUR ) {
        // Compute the inter-frame motion 
        CTrack::Homography M; 
        CTrack::Homography HPrevInv = pTrackingResults->GetPrevHomography(); HPrevInv.inv();
        M.mult( HPrevInv );
        M.logm();
        // DO NOT MULT, the Jacobian needs the external value M.mult( pTrackingResults->GetBlurMagn() );
        
        // For now treat the special case M == 0 differently (FIX ME)
        if( M.m_mH.lpNorm<Eigen::Infinity>() < 1e-6 ) {
            M.zero();
            M.Set(0,0,10);
        }
        ESMJacobianBlur<typename TrackingFunctor::LJ>
            ( pRefPatch, pRefPatchGradX, pRefPatchGradY,
              pRefPatchWTGradX, pRefPatchWTGradY,
              pCurPatch, pCurPatchGradX, pCurPatchGradY,
              nPatchWidth, nPatchHeight, nPatchWidthStep,
              M.GetRowMajorPtr(), pTrackingResults->GetBlurMagn(),
              JtJ, JtE, dRMS );
    }
    else {
        ESMJacobian<typename TrackingFunctor::LJ>
            ( pRefPatch, pRefPatchGradX, pRefPatchGradY,
              pCurPatch, pCurPatchGradX, pCurPatchGradY,
              nPatchWidth, nPatchHeight, nPatchWidthStep,
              JtJ, JtE, dRMS );
    }
    
    // 3') Regularise
    if( bRegularise ) {
        TrackingFunctor::Regularise( TrackingFunctor::LJ::DOF, 
                                     nPatchWidth*nPatchHeight, ///<Input: Number of pixels used in the minimisation FIX ME: when using a mask
                                     JtJ, JtE, pTrackingResults,
                                     dRMS ///<Input/Output: RMS from the dNumPixels reprojections
                                     );
    }

    // 4) Compute and apply update
    bSuccess = TrackingFunctor::UpdateResult( pTrackingResults,
                                              JtJ, JtE, dNormUpdate );
        

    return bSuccess;
}

#endif

