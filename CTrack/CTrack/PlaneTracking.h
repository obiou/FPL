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

#ifndef CTRACK_PLANE_TRACKING_H

#define CTRACK_PLANE_TRACKING_H

#include <iomanip>

#include <CTrack/ImagePyramid.h>
#include <CTrack/TrackingUpdates.h>
#include <CTrack/Types.h>

#define DEBUG_TRACKING 0

#ifdef DEBUG_TRACKING
#  include <CTrack/Misc.h>
#endif

namespace CTrack {
    ////////////////////////////////////////////////////////////////////////////
    //// Simplified call when not tracking with blur
    template< class TrackingFunctor, class MaskFunctorT >
    void TrackPlaneHomography( TrackingSettings<TrackingFunctor,MaskFunctorT>& trackingSettings, ///< Input
                               const ImageHolder& refPatchHolder, ///< Input
                               const ImageHolder& curImageHolder, ///< Input
                               TrackingStats* pTrackingStats,     ///< Output:
                               TrackingResults* pTrackingResults  ///< Output:
                               ) {
        ImageHolder refImageHolder( NULL, 0, 0, 0);
        assert( !TrackingFunctor::BLUR );
        TrackPlaneHomography< TrackingFunctor, MaskFunctorT >
            ( 
             trackingSettings,
             refImageHolder, refPatchHolder,
             curImageHolder, 
             pTrackingStats, pTrackingResults
              );
    }

    ////////////////////////////////////////////////////////////////////////////
    template< class TrackingFunctor, class MaskFunctorT >
    void TrackPlaneHomography( TrackingSettings<TrackingFunctor,MaskFunctorT>& trackingSettings, ///< Input
                               const ImageHolder& refImageHolder, ///< Input: only used if tracking with blur, null buffers are ok otherwise
                               const ImageHolder& refPatchHolder, ///< Input
                               const ImageHolder& curImageHolder, ///< Input
                               TrackingStats* pTrackingStats,     ///< Output:
                               TrackingResults* pTrackingResults  ///< Output:
                               ) {
        // Copy current values to previous values
        // The effect is a bit subtle when looking at the regulariser AND pyramid levels,
        // it should not have a very big effect though...
        pTrackingResults->CopyCurToPrevIgnoringH();

        const int nPatchWidth     = refPatchHolder.nWidth;
        const int nPatchHeight    = refPatchHolder.nHeight;
        const int nPatchWidthStep = refPatchHolder.nWidthStep;
        const int nImageWidth     = curImageHolder.nWidth;
        const int nImageHeight    = curImageHolder.nHeight;
        const int nImageWidthStep = curImageHolder.nWidthStep;

        float* pRefPatchGradX    = trackingSettings.GetTrackBuffer()->RefPatchGradX();
        float* pRefPatchGradY    = trackingSettings.GetTrackBuffer()->RefPatchGradY();
        float* pRefPatchWTGradX  = trackingSettings.GetTrackBuffer()->RefPatchWTGradX();
        float* pRefPatchWTGradY  = trackingSettings.GetTrackBuffer()->RefPatchWTGradY();
        unsigned char* pCurPatch = trackingSettings.GetTrackBuffer()->CurPatch();
        float* pCurPatchGradX    = trackingSettings.GetTrackBuffer()->CurPatchGradX();
        float* pCurPatchGradY    = trackingSettings.GetTrackBuffer()->CurPatchGradY();

        // Compute gradient of reference patch
        CTrack::Gradient( refPatchHolder.pImageData,
                          nPatchWidth, nPatchHeight, nPatchWidthStep,
                          pRefPatchGradX, pRefPatchGradY );

        double dPrevRMS = 0.;
        double dBestRMS = 0.;
        double dNormUpdate = 0.;

        const int nMaxIter = trackingSettings.MaxNumIterations();
        double dMinNormUpdate = 0.1;
        int nNumIter = 0;
        bool bSuccess = true;

        TrackingResults trackingResultsBest = *pTrackingResults;

        TrackingResults trackingResultsPrev = *pTrackingResults;
        TrackingResults trackingResultsCur  = *pTrackingResults;

        bSuccess = SSDUpdateSL3Motion<TrackingFunctor,MaskFunctorT>
            ( refPatchHolder.pImageData,
              pRefPatchGradX, pRefPatchGradY,
              pRefPatchWTGradX, pRefPatchWTGradY,
              nPatchWidth, nPatchHeight, nPatchWidthStep,
              refImageHolder.pImageData,
              curImageHolder.pImageData, 
              nImageWidth, nImageHeight, nImageWidthStep,
              trackingSettings.BegX(),
              trackingSettings.BegY(),
              trackingSettings.Regularise(),
              pCurPatch,
              pCurPatchGradX, pCurPatchGradY,
              &trackingResultsCur,
              &dPrevRMS, &dNormUpdate, 
              trackingSettings.GetMaskFctrRef()
              );
       
        if( !bSuccess ) {
#if DEBUG_TRACKING
            std::cout << "Problem when computing SL3 motion." << std::endl;
#endif
            return;
        }

#if DEBUG_TRACKING
        std::cout << "Initial RMS: " << std::setprecision(14) << dPrevRMS
                  << ", alpha: " << trackingResultsPrev.GetAlpha()
                  << ", beta: " << trackingResultsPrev.GetBeta();
        if( TrackingFunctor::BLUR )  {
            std::cout << ", lambda: " << trackingResultsPrev.GetBlurMagn();
        }
        std::cout << std::endl;
#endif

        dBestRMS = dPrevRMS;
        trackingResultsBest = trackingResultsPrev;
        trackingResultsPrev = trackingResultsCur;

        for( nNumIter=0; nNumIter < nMaxIter; nNumIter++ ) {
#if DEBUG_TRACKING
            double t0 = Tic();
#endif
           
            bSuccess = SSDUpdateSL3Motion<TrackingFunctor,MaskFunctorT>
                ( 
                 refPatchHolder.pImageData, 
                 pRefPatchGradX, pRefPatchGradY,
                 pRefPatchWTGradX, pRefPatchWTGradY,
                 nPatchWidth, nPatchHeight, nPatchWidthStep,
                 refImageHolder.pImageData,
                 curImageHolder.pImageData, 
                 nImageWidth, nImageHeight, nImageWidthStep,
                 trackingSettings.BegX(),
                 trackingSettings.BegY(),
                 trackingSettings.Regularise(),
                 pCurPatch,
                 pCurPatchGradX, pCurPatchGradY,
                 &trackingResultsCur,
                 &dPrevRMS, &dNormUpdate, 
                 trackingSettings.GetMaskFctrRef()
                  ); 

            if( !bSuccess ) {
#if DEBUG_TRACKING
                std::cout << "Problem when computing SL3 motion." << std::endl;
#endif
                break;
            }

            if( dPrevRMS > dBestRMS ) {
#if DEBUG_TRACKING
                std::cout << "RMS increasing from " << dBestRMS << " to " << dPrevRMS << std::endl;
#endif
                break;
            }

            if( dNormUpdate < dMinNormUpdate ) {
#if DEBUG_TRACKING
                std::cout << "RMS decreased too slowly by: " << dBestRMS - dPrevRMS << " to: " << dPrevRMS << std::endl;
#endif
                 if( dPrevRMS < dBestRMS ) {
                     dBestRMS = dPrevRMS;
                     trackingResultsBest = trackingResultsPrev;
                 }
                break;
            }

#if DEBUG_TRACKING
            std::cout << "dRMS: " << dPrevRMS
                      << ", alpha: " << trackingResultsPrev.GetAlpha()
                      << ", beta: " << trackingResultsPrev.GetBeta();
            if( TrackingFunctor::BLUR ) {
                std::cout << ", lambda: " << trackingResultsPrev.GetBlurMagn();
            }
            std:: cout << " / Time: " <<  TocMS( t0 ) << "ms";
            //std:: cout << " / dNormUpdate: " << dNormUpdate;
            std::cout << std::endl;
#endif
            if( dPrevRMS < dBestRMS ) {
                dBestRMS = dPrevRMS;
                trackingResultsBest = trackingResultsPrev;
            }
            trackingResultsPrev = trackingResultsCur;
        }

        trackingResultsCur = trackingResultsBest;
        bSuccess = SSDUpdateSL3Motion<TrackingFunctor,MaskFunctorT>
            ( refPatchHolder.pImageData, 
              pRefPatchGradX, pRefPatchGradY,
              pRefPatchWTGradX, pRefPatchWTGradY,
              nPatchWidth, nPatchHeight, nPatchWidthStep,
              refImageHolder.pImageData,
              curImageHolder.pImageData, 
              nImageWidth, nImageHeight, nImageWidthStep,
              trackingSettings.BegX(),
              trackingSettings.BegY(),
              trackingSettings.Regularise(),
              pCurPatch,
              pCurPatchGradX, pCurPatchGradY,
              // Warning: this will be
              // updated (use something different than trackingResultsBest
              &trackingResultsCur,
              &dBestRMS, &dNormUpdate, 
              trackingSettings.GetMaskFctrRef()
              );
            
#if DEBUG_TRACKING
        std::cout << "Number of iterations: " << nNumIter << std::endl;
        std::cout << "Final RMS: " << dBestRMS << std::endl;
        std::cout << "Best estimates: " << trackingResultsBest << std::endl;
#endif

        pTrackingStats->RMS( dBestRMS );
        pTrackingStats->NumIter( nNumIter );
        *pTrackingResults = trackingResultsBest;
    }
  
    ////////////////////////////////////////////////////////////////////////////
    //// Simplified call when not tracking with blur
    template< class TrackingFunctor, class MaskFunctorT >
    void TrackPlaneHomographyPyr( TrackingSettingsPyr<TrackingFunctor,MaskFunctorT>& trackingSettings, ///< Input
                                  ImagePyramid& refPatchPyramid, ///< Input
                                  ImagePyramid& curImagePyramid, ///< Input
                                  TrackingStatsPyr* pTrackingStatsPyr, ///< Output:
                                  TrackingResults* pTrackingResults    ///< Output:
                                  ) {
        ImagePyramid refImagePyramid;
        TrackPlaneHomographyPyr< TrackingFunctor, MaskFunctorT>
            ( trackingSettings,
              refImagePyramid, refPatchPyramid,
              curImagePyramid, 
              pTrackingStatsPyr, pTrackingResults
              );
    }

    ////////////////////////////////////////////////////////////////////////////
    template< class TrackingFunctor, class MaskFunctorT >
    void TrackPlaneHomographyPyr( TrackingSettingsPyr<TrackingFunctor,MaskFunctorT>& trackingSettings, ///< Input
                                  ImagePyramid& refImagePyramid, ///< Input
                                  ImagePyramid& refPatchPyramid, ///< Input
                                  ImagePyramid& curImagePyramid, ///< Input
                                  TrackingStatsPyr* pTrackingStatsPyr, ///< Output:
                                  TrackingResults* pTrackingResults    ///< Output:
                                  ) {
        for( int nLevel=trackingSettings.GetNumLevels()-1; nLevel>=0; nLevel-- ) {
#if DEBUG_TRACKING
            std::cout << "--- Level " << nLevel << std::endl;
#endif
            // Scale up/down completely at each step to allow early backout
            double dScale = 1 << nLevel;

            pTrackingResults->SetHomography( pTrackingResults->GetHomography().scale( dScale ) );

            if( TrackingFunctor::BLUR ) {
                pTrackingResults->SetPrevHomography( pTrackingResults->GetPrevHomography().scale( dScale ) );
            }

            if( TrackingFunctor::BLUR ) {
                TrackPlaneHomography<TrackingFunctor,MaskFunctorT>
                    ( *trackingSettings.GetTrackingSettings( nLevel ),
                      *refImagePyramid.GetImage( nLevel ),
                      *refPatchPyramid.GetImage( nLevel ),
                      *curImagePyramid.GetImage( nLevel ),
                      pTrackingStatsPyr->GetTrackingStats( nLevel ),
                      pTrackingResults
                      );
            }
            else { 
                TrackPlaneHomography<TrackingFunctor,MaskFunctorT>
                    ( *trackingSettings.GetTrackingSettings( nLevel ),
                      *refPatchPyramid.GetImage( nLevel ),
                      *curImagePyramid.GetImage( nLevel ),
                      pTrackingStatsPyr->GetTrackingStats( nLevel ),
                      pTrackingResults
                      );
            }
            // Scale back up
            pTrackingResults->SetHomography( pTrackingResults->GetHomography().scale( 1./dScale ) );
            
            if( TrackingFunctor::BLUR ) {
                pTrackingResults->SetPrevHomography( pTrackingResults->GetPrevHomography().scale( 1./dScale ) );
             }
        }
    }

    ////////////////////////////////////////////////////////////////////////////
}

#endif
