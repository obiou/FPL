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

///
/// Contains different operations applied to the image: warping,
/// simple pyramid, change in illumination.
/// The functions here use other methods from the CTrack library.
/// in constrast to @see Transforms.h
///
/// These are low-level calls with minimal checks on sizes. There
/// should be no reason to use these functions directly.
///
/// \file TransformsHighLevel.h
///
#ifndef CTRACK_TRANSFORMS_HIGH_LEVEL_H

#define CTRACK_TRANSFORMS_HIGH_LEVEL_H

#include <iostream>
#include <limits>
#include <math.h>

#include <CTrack/Homography.h>

namespace CTrack {
    ////////////////////////////////////////////////////////////////////////////
    /// Warps an image patch according to a homography H and blurs using a matrix M:
    /// This procedure is inefficient as it computes multiple warps instead of summing along the streamline (see CVPR08)
    /// TIBlur( x ) = \int_0^1 I( Hl*expm( -t*M )*Hr x ) dt.
    template< class MaskFunctorT >
        void WarpBlur( const unsigned char* pImage, ///< Input: image input
                       const int nImageWidth,       ///< Input: image width
                       const int nImageHeight,      ///< Input: image height
                       const int nImageWidthStep,   ///< Input: image step to change row for non contiguous memory (eg OpenCV)
                       const Homography* pHl,          ///< Input: 3x3 homography matrix in row-major
                       const Homography* pM,           ///< Input: 3x3 blur matrix (Lie algebra value correponding to a constant velocity model), matrix in row-major
                       const Homography* pHr,          ///< Input: 3x3 homography matrix in row-major
                       unsigned char* pWarpedPatch, ///< Output: memory allocated by the user that will be used to store the result of the warping
                       const int nPatchWidth,       ///< Input: warped patch width
                       const int nPatchHeight,      ///< Input: warped patch height
                       const int nPatchWidthStep,   ///< Input: warped patch step to change row for non contiguous memory (eg OpenCV)
                       MaskFunctorT& maskFctr, ///< Output: memory allocated by the user that will contain the bytes to ignore (out-of-image) 1 <-> ok
                       double dStep = 0.9, /// Input: step in pixels between samples (tradeoff between speed and accuracy)
                       bool bJacobian = false  ///< Input: should the Jacobian be computed (instead of just the warp)
                       );
}

////////////////////////////////////////////////////////////////////////////////
// Computes p1 = p1 + dScale*p2
namespace CTrack{
    inline void addScaledImage( float* p1, unsigned char* p2, float fScale,
                                const int nWidth, const int nHeight ) 
    {
        for( int y=0; y<nHeight; y++ ) {
            for( int x=0; x<nWidth; x++ ) {
                *p1 = *p1 + fScale*static_cast<float>(*p2);
                p1++;
                p2++;
            }
        }
    }
}

////////////////////////////////////////////////////////////////////////////////
template< class MaskFunctorT >
void CTrack::WarpBlur( const unsigned char* pImage, ///< Input: image input
                       const int nImageWidth,       ///< Input: image width
                       const int nImageHeight,      ///< Input: image height
                       const int nImageWidthStep,   ///< Input: image step to change row for non contiguous memory (eg OpenCV)
                       const Homography* pHl,          ///< Input: 3x3 homography matrix in row-major
                       const Homography* pM,           ///< Input: 3x3 blur matrix (Lie algebra value correponding to a constant velocity model), matrix in row-major
                       const Homography* pHr,          ///< Input: 3x3 homography matrix in row-major
                       unsigned char* pWarpedBlurredPatch, ///< Output: memory allocated by the user that will be used to store the result of the warping
                       const int nPatchWidth,       ///< Input: warped patch width
                       const int nPatchHeight,      ///< Input: warped patch height
                       const int nPatchWidthStep,   ///< Input: warped patch step to change row for non contiguous memory (eg OpenCV)
                       MaskFunctorT& maskFctr, ///< Output: memory allocated by the user that will contain the bytes to ignore (out-of-image) 1 <-> ok
                       double dStep = 1, /// Input: step in pixels between samples (tradeoff between speed and accuracy)
                       bool bJacobian = false  ///< Input: should the Jacobian be computed (instead of just the warp)
                       )
{
    const int nPadding = nPatchWidthStep - nPatchWidth;

    // Compute increment magnitude by sampling from the corner points and center point
    std::vector<std::pair<double,double> > vPolyIn;
    // vPolyOut will contain the sample points warped by the blur (psf end points)
    std::vector<std::pair<double,double> > vPolyOut;

    typedef std::pair<double,double> Pair;
    vPolyIn.push_back( Pair( 0, 0 ) );
    vPolyIn.push_back( Pair( nImageWidth-1, 0 ) );
    vPolyIn.push_back( Pair( nImageWidth-1, nImageHeight-1 ) );
    vPolyIn.push_back( Pair( 0, nImageHeight-1 ) );
    vPolyIn.push_back( Pair( nImageWidth/2, nImageHeight/2 ) );

    pM->warp_polymult_exp( vPolyIn, vPolyOut ); // warp by expm( M )

    // Find the max value to choose a sampling step for the blur
    double dMaxDisplacement = 0.;
    for( size_t ii=0; ii<vPolyIn.size(); ii++ ) {
        dMaxDisplacement = std::max( dMaxDisplacement,
                                     std::abs(vPolyIn[ii].first - vPolyOut[ii].first) );
        dMaxDisplacement = std::max( dMaxDisplacement,
                                     std::abs(vPolyIn[ii].second - vPolyOut[ii].second) );       
    }

    float fFact;
    
    if( dMaxDisplacement != 0 ) {
        fFact = static_cast<float>(dStep/dMaxDisplacement);
    }
    else {
        fFact = 1;
    }
    if( fFact > 1 || fFact <= 1e-100 ) {
        fFact = 1;
    }
#if 0
    std::cout << "Sampling factor for blurring: " 
              << fFact 
              << ", dStep: " << dStep
              << ", dMaxDisplacement: " << dMaxDisplacement << "."
              << std::endl;
#endif

    /// Compute blur
    // Create a buffer to accumulate values
    float pTmpBlurred[ nPatchWidth*nPatchHeight ];
    memset( pTmpBlurred, 0, nPatchWidth*nPatchHeight*sizeof(float) );
 
    // Create a buffer for storing warped images
    unsigned char pTmpWarped[ nPatchWidth*nPatchHeight ];

    float fMult = 0;
    Homography mH;
    Homography mExpMultM;

    float fNumComp = 0;
    
    while( fMult <= 1 ) {
        // Compute warping homography H = Hl*expm( fMult*M )*Hr
        mH.id();
        mH.mult( *pHl );
        mExpMultM = *pM;
        mExpMultM.mult( fMult );
        mExpMultM.expm();
        mH.mult( mExpMultM );
        mH.mult( *pHr );

        //std::cout << mH.m_mH << std::endl << std::endl;

        // Compute warp for H
        Warp<>( pImage, nImageWidth, nImageHeight, nImageWidthStep,
                mH.GetRowMajorPtr(), // Current value for fMult  
                pTmpWarped, nPatchWidth, nPatchHeight, 
                nPatchWidth, maskFctr // working on buffer without an widthStep
                );

        // Compute update
        if( bJacobian ) {
            // Trapezium rule
            if( fMult == 0 || fMult == 1 ) { 
                addScaledImage( pTmpBlurred, pTmpWarped, 0.5f*fMult,
                                nPatchWidth, nPatchHeight );
            } 
            else {
                addScaledImage( pTmpBlurred, pTmpWarped, fMult,
                                nPatchWidth, nPatchHeight );
            }
        }
        else {
            // Trapezium rule
            if( fMult == 0 || fMult + fFact > 1 ) {
                addScaledImage( pTmpBlurred, pTmpWarped, 0.5f,
                                nPatchWidth, nPatchHeight );
            }
            else {
                addScaledImage( pTmpBlurred, pTmpWarped, 1,
                                nPatchWidth, nPatchHeight );
            }
        }
        fNumComp++;
        fMult += fFact;
    }
    fNumComp--; // only count once for beg and end
    // Set the values in pWarpedBlurredPatch by dividing all terms in
    // the temporary accumulation buffer by the number of samples
    float* pTmp = pTmpBlurred;
    for( int y=0; y<nPatchHeight; y++ ) {
        for( int x=0; x<nPatchWidth; x++ ) {
            *pWarpedBlurredPatch++ = (char) (*pTmp++/fNumComp);
        }
        for( int ii=0; ii < nPadding; ii++ ) {
            pWarpedBlurredPatch++;
        }
    }
}

#endif
