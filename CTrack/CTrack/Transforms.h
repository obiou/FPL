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
/// Contains the different operations applied to the image: warping, simple pyramid, change in illumination.
/// These are low-level calls with minimal checks on sizes. There should be no reason to use these functions directly.
///
/// \file Transforms.h
///
#ifndef CTRACK_TRANSFORMS_H

#define CTRACK_TRANSFORMS_H

#include <ctrack_includes.h>

#if CTRACK_HAS_OPENCV
#  include <cv.h>
#  include <highgui.h>
#endif

#include <iomanip>
#include <iostream>
#include <limits>
#include <math.h>

#ifdef CTRACK_USE_SSSE3
#  include <tmmintrin.h>
#endif

#include <CTrack/Masks.h>

namespace CTrack {
#if CTRACK_HAS_OPENCV
    ////////////////////////////////////////////////////////////////////////////
    template< class MaskFunctorT >
        void Warp( const IplImage* pImage, ///< Input: image input
                   const double*  pH,       ///< Input: 3x3 homography matrix in row-major
                   IplImage* pWarpedPatch,  ///< Output: memory allocated by the user that will be used to store the result of the warping
                   MaskFunctorT& maskFctr   ///< Output: memory allocated by the user that will contain the bytes to ignore (out-of-image) 1 <-> ok
                   );

    ////////////////////////////////////////////////////////////////////////////
    void Warp( const IplImage* pImage, ///< Input: image input
               const double*  pH,      ///< Input: 3x3 homography matrix in row-major
               IplImage* pWarpedPatch  ///< Output: memory allocated by the user that will be used to store the result of the warping
               );
#endif

    //////////////////////////////////////////////////////////////////////////////////////
    /// Warps a patch of size "nPatchWidth x nPatchHeight" to image pImage using the homography H.
    /// (ie. H * pWarpedPatch =  pImage )
    /// The patch coordinates are taken at the top left of the patch (NOT centered).
    /// H must include the patch position in the image and is in row-major.
    /// For non-contiguous memory (eg OpenCV images) please use the other warp function.
    ///
    /// WARNING: if using col-major form (e.g. Eigen), do not forget to transpose H.
    template< class MaskFunctorT >
        void Warp( const unsigned char* pImage,  ///< Input: image input
                   const int nImageWidth,     ///< Input: image width
                   const int nImageHeight,    ///< Input: image height
                   const double*  pH,         ///< Input: 3x3 homography matrix in row-major
                   unsigned char* pWarpedPatch,     ///< Output: memory allocated by the user that will be used to store the result of the warping
                   const int nPatchWidth,    ///< Input: warped patch width
                   const int nPatchHeight,   ///< Input: warped patch height
                   MaskFunctorT& maskFctr     ///< Output: memory allocated by the user that will contain the bytes to ignore (out-of-image) 1 <-> ok
                   );

    //////////////////////////////////////////////////////////////////////////////////////
    /// Warps a patch of size "nPatchWidth x nPatchHeight" to image pImage using the homography H.
    /// (ie. H * pWarpedPatch =  pImage )
    /// The patch coordinates are taken at the top left of the patch (NOT centered).
    /// H must include the patch position in the image and is in row-major.
    /// This call ignores out-of-image values.
    ///
    /// WARNING: if using col-major form (e.g. Eigen), do not forget to transpose H.
    void Warp( const unsigned char* pImage, ///< Input: image input
               const int nImageWidth,    ///< Input: image width
               const int nImageHeight,   ///< Input: image height
               const int nImageWidthStep,
               const double*  pdH,        ///< Input: 3x3 homography matrix in row-major
               unsigned char* pWarpedPatch,     ///< Input/Output: memory allocated by the user that will be used to store the result of the warping
               const int nPatchWidth,     ///< Input: warped patch width
               const int nPatchHeight,    ///< Input: warped patch height
               const int nPatchWidthStep 
               );

    void Warp( const unsigned char* pImage, ///< Input: image input
               const int nImageWidth,    ///< Input: image width
               const int nImageHeight,   ///< Input: image height
               const double*  pdH,        ///< Input: 3x3 homography matrix in row-major
               unsigned char* pWarpedPatch,     ///< Input/Output: memory allocated by the user that will be used to store the result of the warping
               const int nPatchWidth,     ///< Input: warped patch width
               const int nPatchHeight    ///< Input: warped patch height
               );

    ///////////////////////////////////////////////////////////////////////////////////////////////
    /// Warps a patch of size "nPatchWidth x nPatchHeight" to image pImage using the homography H.
    /// (ie. H * pWarpedPatch =  pImage )
    /// The patch coordinates are taken at the top left of the patch (NOT centered).
    /// H must include the patch position in the image and is in row-major.
    /// This call can be used on non-contiguous memory (eg OpenCV images) by specifying the width step.
    ///
    /// WARNING: if using col-major form (e.g. Eigen), do not forget to transpose H.
    template< class MaskFunctorT >
        void Warp( const unsigned char* pImage,  ///< Input: image input
                   const int nImageWidth,     ///< Input: image width
                   const int nImageHeight,    ///< Input: image height
                   const int nImageWidthStep, ///< Input: image step to change row for non contiguous memory (eg OpenCV)
                   const double*  pH,         ///< Input: 3x3 homography matrix in row-major
                   unsigned char* pWarpedPatch,     ///< Output: memory allocated by the user that will be used to store the result of the warping
                   const int nPatchWidth,    ///< Input: warped patch width
                   const int nPatchHeight,   ///< Input: warped patch height
                   const int nPatchWidthStep, ///< Input: warped patch step to change row for non contiguous memory (eg OpenCV)
                   MaskFunctorT& maskFctr ///< Output: memory allocated by the user that will contain the bytes to ignore (out-of-image) 1 <-> ok
                   );
    
    ///////////////////////////////////////////////////////////////////////////////////////////////
    /// Warps a patch of size "nPatchWidth x nPatchHeight" to image pImage using the homography H
    /// and applies an illumination model transform ( fAlpha*pPath + fBeta )
    /// (ie. H * pWarpedPatch =  pImage )
    /// The patch coordinates are taken at the top left of the patch (NOT centered).
    /// H must include the patch position in the image and is in row-major.
    /// This call can be used on non-contiguous memory (eg OpenCV images) by specifying the width step.
    ///
    /// WARNING: if using col-major form (e.g. Eigen), do not forget to transpose H.
    template< class MaskFunctorT >
        void WarpIllum( const unsigned char* pImage, ///< Input: image input
                        const int nImageWidth,    ///< Input: image width
                        const int nImageHeight,   ///< Input: image height
                        const int nImageWidthStep,
                        const double*  pdH,        ///< Input: 3x3 homography matrix in row-major
                        const float fAlpha, ///< Multiplicative part of the affine illumination model
                        const float fBeta, ///< Additive part of the affine illumination model
                        unsigned char* pWarpedPatch,     ///< Input/Output: memory allocated by the user that will be used to store the result of the warping
                        const int nPatchWidth,     ///< Input: warped patch width
                        const int nPatchHeight,    ///< Input: warped patch height
                        const int nPatchWidthStep, 
                        MaskFunctorT& maskFctr ///< Input/Output: functor that can be used to store the bytes to ignore (out-of-image)
                        );
 
    ////////////////////////////////////////////////////////////////////////////////
    template< class MaskFunctorT >
        void WarpTranslation( const unsigned char* pImage, ///< Input: image input
                              const int nImageWidth,    ///< Input: image width
                              const int nImageHeight,   ///< Input: image height
                              float dfX, ///< Input: offset in X
                              float dfY, ///< Input: offset in Y
                              unsigned char* pWarpedPatch,     ///< Input/Output: memory allocated by the user that will be used to store the result of the warping
                              const int nPatchWidth,     ///< Input: warped patch width
                              const int nPatchHeight,    ///< Input: warped patch height
                              MaskFunctorT& maskFctr ///< Input/Output: memory allocated by the user that will contain the bytes to ignore (out-of-image)
                              );

    ////////////////////////////////////////////////////////////////////////////////
    void WarpTranslation( const unsigned char* pImage, ///< Input: image input
                          const int nImageWidth,    ///< Input: image width
                          const int nImageHeight,   ///< Input: image height
                          float dfX, ///< Input: offset in X
                          float dfY, ///< Input: offset in Y
                          unsigned char* pWarpedPatch,     ///< Input/Output: memory allocated by the user that will be used to store the result of the warping
                          const int nPatchWidth,     ///< Input: warped patch width
                          const int nPatchHeight    ///< Input: warped patch height
                          );

    ////////////////////////////////////////////////////////////////////////////////
    template< class MaskFunctorT >
        void WarpTranslation( const unsigned char* pImage, ///< Input: image input
                              const int nImageWidth,    ///< Input: image width
                              const int nImageHeight,   ///< Input: image height
                              const int nImageWidthStep,
                              float dfX, ///< Input: offset in X
                              float dfY, ///< Input: offset in Y
                              unsigned char* pWarpedPatch,     ///< Input/Output: memory allocated by the user that will be used to store the result of the warping
                              const int nPatchWidth,     ///< Input: warped patch width
                              const int nPatchHeight,    ///< Input: warped patch height
                              const int nPatchWidthStep, 
                              MaskFunctorT& maskFctr ///< Input/Output: memory allocated by the user that will contain the bytes to ignore (out-of-image)
                              );

    ////////////////////////////////////////////////////////////////////////////////
    template< class MaskFunctorT >
        void WarpTranslationIllum( const unsigned char* pImage, ///< Input: image input
                                   const int nImageWidth,    ///< Input: image width
                                   const int nImageHeight,   ///< Input: image height
                                   const int nImageWidthStep,
                                   float dfX, ///< Input: offset in X
                                   float dfY, ///< Input: offset in Y
                                   float fAlpha, float fBeta,
                                   unsigned char* pWarpedPatch,     ///< Input/Output: memory allocated by the user that will be used to store the result of the warping
                                   const int nPatchWidth,     ///< Input: warped patch width
                                   const int nPatchHeight,    ///< Input: warped patch height
                                   const int nPatchWidthStep, 
                                   MaskFunctorT& maskFctr
                                   ); 

    ////////////////////////////////////////////////////////////////////////////////
    void WarpTranslation( const unsigned char* pImage, ///< Input: image input
                          const int nImageWidth,    ///< Input: image width
                          const int nImageHeight,   ///< Input: image height
                          const int nImageWidthStep,
                          float dfX, ///< Input: offset in X
                          float dfY, ///< Input: offset in Y
                          unsigned char* pWarpedPatch,     ///< Input/Output: memory allocated by the user that will be used to store the result of the warping
                          const int nPatchWidth,     ///< Input: warped patch width
                          const int nPatchHeight,    ///< Input: warped patch height
                          const int nPatchWidthStep
                          );

    //////////////////////////////////////////////////////////////////////////////
    /// Decimate image by 2, the image size has to be dividable by two or false is returned.
    /// WARNING: the memory also has to be continuous (widthStep==width in OpenCV)
    bool HalveImage( const unsigned char *pInputData, ///< Input: input image
                     const int nWidth,                ///< Input: image width
                     const int nHeight,               ///< Input: image height
                     unsigned char *pOutputData       ///< Output: decimated image
                     );
    
    /////////////////////////////////////////////////////////////////////////////
    /// Decimate image by 2, the image size has to be dividable by two or false is returned.
    /// The 'widthStep' arguments make it possible to use non continuous memory as in OpenCV.
    bool HalveImage( const unsigned char *pInputData, ///< Input: input image
                     const int nWidth,                ///< Input: image width
                     const int nHeight,               ///< Input: image height
                     const int nWidthStep,            ///< Input: image step to change row for non contiguous memory (eg OpenCV)
                     unsigned char *pOutputData,      ///< Output: decimated image
                     const int nOutputWidthStep       ///< Input: decimated image step to change row for non contiguous memory (eg OpenCV)
                     );

#ifdef CTRACK_USE_SSSE3
    /////////////////////////////////////////////////////////////////////////////
    void HalveImageSSSE3( const unsigned char *pInputData, 
                          const int nWidth,                
                          const int nHeight,               
                          unsigned char *pOutputData       
                          )
    {
        // Check image size is dividable by 2
        assert( 2*(nWidth >> 1) == nWidth && 2*(nHeight >> 1) == nHeight);
        // Check image width is dividable by 16
        assert( 16*(nWidth >> 4) == nWidth );
        
        int irowMnWidth, irowP1MnWidth;

        __m128i avg, avgShift;
        __m128i mask = _mm_set_epi8( 0,0,0,0,0,0,0,0,15,13,11,9,7,5,3,1 );

        for( int irow = 0; irow < nHeight-1; irow+=2 ) {
            irowMnWidth   = irow*nWidth;
            irowP1MnWidth = irowMnWidth+nWidth;
            for( int jcol = 0; jcol < nWidth-15; jcol+=16 ) {
                avg = _mm_avg_epu8( ((__m128i*) &pInputData[ irowMnWidth + jcol ])[0],
                                    ((__m128i*) &pInputData[ irowP1MnWidth + jcol ])[0] );
                avgShift = _mm_slli_si128( avg, 1 );
                avg = _mm_avg_epu8( avg, avgShift );
                avgShift = _mm_shuffle_epi8( avg, mask );

                _mm_storel_epi64( (__m128i*) pOutputData, avgShift );
                pOutputData += 8;
            }
        }
    }
#else
    /////////////////////////////////////////////////////////////////////////////
    void HalveImageSSSE3( const unsigned char *pInputData, 
                          const int nWidth,                
                          const int nHeight,               
                          unsigned char *pOutputData       
                          )
    {
        //    printf("No ssse3\n");
        HalveImage( pInputData, nWidth, nHeight, pOutputData );
    }
#endif // CTRACK_USE_SSSE3  

    /////////////////////////////////////////////////////////////////////////////
    /// Compute a simple gradient [-1 0 1] in x and y directions (fast)
    void Gradient( unsigned char* pImage, ///< Input:
                   const int nImageWidth,       ///< Input:
                   const int nImageHeight,      ///< Input:     
                   float* pGradX,               ///< Output:
                   float* pGradY                ///< Output:
                   );

    /////////////////////////////////////////////////////////////////////////////
    /// Compute a simple gradient [-1 0 1] in x and y directions (fast)
    void Gradient( unsigned char* pImage, ///< Input:
                   const int nImageWidth,       ///< Input:
                   const int nImageHeight,      ///< Input:     
                   const int nImageWidthStep,
                   float* pGradX,               ///< Output:
                   float* pGradY                ///< Output:
                   );

    ////////////////////////////////////////////////////////////////////////////
    double RMS( unsigned char* pIm1, unsigned char* pIm2,
                const int nWidth, const int nHeight, 
                const int nWidthStep1, const int nWidthStep2 ) {
        double dRMS = 0.;

        for( int y=0; y<nHeight; y++ ) {
            for( int x=0; x<nWidth; x++ ) {
                double dDiff = double(*pIm1++) - double(*pIm2++);
                dRMS += dDiff*dDiff;
            }
            for( int ii=0; ii < nWidthStep1 - nWidth; ii++ ) {
                pIm1++; 
            }
            for( int ii=0; ii < nWidthStep2 - nWidth; ii++ ) {
                pIm2++; 
            }
        }
        return sqrt( dRMS/(nWidth*nHeight) );
    }
} 

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
#if CTRACK_HAS_OPENCV
////////////////////////////////////////////////////////////////////////////////
template< class MaskFunctorT >
void CTrack::Warp( const IplImage* pImage,
                   const double*  pH,
                   IplImage* pWarpedPatch,
                   MaskFunctorT& maskFctr 
                   ) {
    Warp( (const unsigned char*)pImage->imageData,
          pImage->width, pImage->height,
          pImage->widthStep,
          pH,
          (unsigned char*) pWarpedPatch->imageData, 
          pWarpedPatch->width, 
          pWarpedPatch->height, 
          pWarpedPatch->widthStep, maskFctr );
}

////////////////////////////////////////////////////////////////////////////////
void CTrack::Warp( const IplImage* pImage, ///< Input: image input
                   const double*  pH,       ///< Input: 3x3 homography matrix in row-major
                   IplImage* pWarpedPatch  ///< Output: memory allocated by the user that will be used to store the result of the warping
                   ) {
    NoMask noMask;
    CTrack::Warp( pImage, pH, pWarpedPatch, noMask );
}

////////////////////////////////////////////////////////////////////////////////
#endif

////////////////////////////////////////////////////////////////////////////////
void CTrack::Warp( const unsigned char* pImage,  ///< Input: image input
                   const int nImageWidth,     ///< Input: image width
                   const int nImageHeight,    ///< Input: image height
                   const double*  pH,         ///< Input: 3x3 homography matrix in row-major
                   unsigned char* pWarpedPatch,     ///< Input/Output: memory allocated by the user that will be used to store the result of the warping
                   const int nPatchWidth,    ///< Input: warped patch width
                   const int nPatchHeight    ///< Input: warped patch height
                   ) 
{
    NoMask noMask;
    CTrack::Warp( pImage, nImageWidth, nImageHeight, nImageWidth, pH,
                  pWarpedPatch, nPatchWidth, nPatchHeight, nPatchWidth,
                  noMask );
}

////////////////////////////////////////////////////////////////////////////////
template< class MaskFunctorT >
inline void BilinearInterpolation( const int nWM, const int nHM, 
                                   const int nImageWidthStep,
                                   const unsigned char* pImage,
                                   const float fXc, const float fYc,
                                   unsigned char* pWarpedPatch,
                                   MaskFunctorT& maskFctr
                                   )
{
    // Truncates the values
    const int nFloorXc = (int)fXc;
    const int nFloorYc = (int)fYc;

    if( nFloorXc >= 0 &&
        nFloorYc >= 0 &&
        nFloorXc < nWM && 
        nFloorYc < nHM ) {

        // Main case: points inside the image
        int nCoord = nFloorXc + nFloorYc*nImageWidthStep;

        float fV00 = pImage[ nCoord ];
        float fV10 = pImage[ nCoord + 1  ];
        nCoord += nImageWidthStep;
        float fV01 = pImage[ nCoord ];
        float fV11 = pImage[ nCoord + 1 ];

#if 0
        // This is slower with fewer mult ! :
        const float fTmp1 = fV00 + (fXc-nFloorXc)*(fV10-fV00);
        const float fTmp2 = fV01 + (fXc-nFloorXc)*(fV11-fV01);
        //*pWarpedPatch = (char) fTmp1 + (fYc-nFloorYc)*( fTmp2 - fTmp1 );
        *pWarpedPatch = (char) fTmp1 + (fYc-nFloorYc)*fTmp2 - (fYc-nFloorYc)*fTmp1;
#else

#if 0
        *pWarpedPatch = (char) ( fV00 +
                                 (fXc-nFloorXc)*(fV10-fV00)+
                                 (fYc-nFloorYc)*(fV01-fV00)-
                                 (fXc-nFloorXc)*(fYc-nFloorYc)*(fV01-fV00+fV10-fV11) );
#endif

        *pWarpedPatch = static_cast<char>
            ( fV00 +
              (fXc-static_cast<float>(nFloorXc))*(fV10-fV00)+
              (fYc-static_cast<float>(nFloorYc))*(fV01-fV00)-
              (fXc-static_cast<float>(nFloorXc))*(fYc-static_cast<float>(nFloorYc))*(fV01-fV00+fV10-fV11) );

#endif

        //*pWarpedPatchMask = 1;
        maskFctr.set( true );
    } 
    else { 
        //printf( "WARNING: out of image in Warp\n" );
        *pWarpedPatch     = 0;
        //*pWarpedPatchMask = 0;
        maskFctr.set( false );
    }
}

////////////////////////////////////////////////////////////////////////////////
template< class MaskFunctorT >
inline void BilinearInterpolationIllum( const int nWM, const int nHM, 
                                        const int nImageWidthStep,
                                        const unsigned char* pImage,
                                        const float fXc, const float fYc,
                                        const float fAlpha, const float fBeta,
                                        unsigned char* pWarpedPatch,
                                        MaskFunctorT& maskFctr
                                        )
{
    // Truncates the values
    int nFloorXc = (int)fXc;
    float fDx = fXc - static_cast<float>(nFloorXc);
    int nFloorYc = (int)fYc;
    float fDy = fYc - static_cast<float>(nFloorYc);
    
    if( fXc >= 0. &&
        fYc >= 0. &&
        nFloorXc < nWM && 
        nFloorYc < nHM ) {
        
        // Main case: points inside the image
        float fV00 = pImage[nFloorXc   + nFloorYc*nImageWidthStep ];
        float fV10 = pImage[nFloorXc+1 + nFloorYc*nImageWidthStep ];
        float fV01 = pImage[nFloorXc   + (nFloorYc+1)*nImageWidthStep ];
        float fV11 = pImage[nFloorXc+1 + (nFloorYc+1)*nImageWidthStep ];
        
        float fRes = fAlpha*( fV00+fDx*(fV10-fV00)+fDy*(fV01-fV00)-fDy*fDx*((fV01-fV00)+fV10-fV11) ) + fBeta;
        *pWarpedPatch = fRes > 255.f ? char(255) : (fRes < 0.f ? char(0) : static_cast<char>(fRes));
        //*pWarpedPatchMask = 1;
        maskFctr.set( true );
    } 
    else { 
        //printf( "WARNING: out of image in Warp\n" );
        *pWarpedPatch     = 0;
        //*pWarpedPatchMask = 0;
        maskFctr.set( false );
    }
}

////////////////////////////////////////////////////////////////////////////////
void CTrack::Warp( const unsigned char* pImage, ///< Input: image input
                   const int nImageWidth,    ///< Input: image width
                   const int nImageHeight,   ///< Input: image height
                   const int nImageWidthStep,
                   const double*  pdH,        ///< Input: 3x3 homography matrix in row-major
                   unsigned char* pWarpedPatch,     ///< Input/Output: memory allocated by the user that will be used to store the result of the warping
                   const int nPatchWidth,     ///< Input: warped patch width
                   const int nPatchHeight,    ///< Input: warped patch height
                   const int nPatchWidthStep 
                   ) 
{
    NoMask noMask;
    CTrack::Warp( pImage, nImageWidth, nImageHeight, nImageWidthStep,
                  pdH, pWarpedPatch, nPatchWidth, nPatchHeight, 
                  nPatchWidthStep, noMask );
}

////////////////////////////////////////////////////////////////////////////////
template< class MaskFunctorT >
void CTrack::Warp( const unsigned char* pImage, ///< Input: image input
                   const int nImageWidth,    ///< Input: image width
                   const int nImageHeight,   ///< Input: image height
                   const int nImageWidthStep,
                   const double*  pdH,        ///< Input: 3x3 homography matrix in row-major
                   unsigned char* pWarpedPatch,     ///< Input/Output: memory allocated by the user that will be used to store the result of the warping
                   const int nPatchWidth,     ///< Input: warped patch width
                   const int nPatchHeight,    ///< Input: warped patch height
                   const int nPatchWidthStep, 
                   MaskFunctorT& maskFctr ///< Input/Output: functor that can be used to store the bytes to ignore (out-of-image)
                   ) 
{
    const int nPatchPadding = nPatchWidthStep - nPatchWidth;
    const int nWM = nImageWidth - 1;
    const int nHM = nImageHeight - 1;

    float pH[9];
    for( int ii=0; ii < 9; ii++ ) {
        pH[ii] = float( pdH[ii] );
    }
 
    float fXc,fYc,fZc;
    float fPartXc,fPartYc,fPartZc;

    float fH6m, fH0m, fH3m;

    //
    // dColRef,nRowRef: values taken in the reference image
    // fXc,fYc,fZc : values taken in the current image
    // 
    fPartXc = pH[2];
    fPartYc = pH[5];
    fPartZc = pH[8];    
    for( int nRowRef=0; nRowRef<nPatchHeight; nRowRef++ ) {
        fH6m = fPartZc; fH0m = fPartXc; fH3m = fPartYc;
        for( int nColRef=0; nColRef<nPatchWidth; nColRef++ ) {
            fZc = 1/fH6m;
            fXc = fH0m*fZc;
            fYc = fH3m*fZc;

            fH6m += pH[6];
            fH0m += pH[0];
            fH3m += pH[3];    

            BilinearInterpolation( nWM, nHM, nImageWidthStep,
                                   pImage, fXc, fYc, 
                                   pWarpedPatch, maskFctr
                                   );

            //pWarpedPatchMask++;
            maskFctr.fwd();
            pWarpedPatch++;
        }
        fPartXc += pH[1];
        fPartYc += pH[4];
        fPartZc += pH[7];    
        for( int kk = 0; kk < nPatchPadding; kk++ ) {
            pWarpedPatch++;
        }
    }
    maskFctr.end();
}

////////////////////////////////////////////////////////////////////////////////
template< class MaskFunctorT >
void CTrack::WarpIllum( const unsigned char* pImage, ///< Input: image input
                        const int nImageWidth,    ///< Input: image width
                        const int nImageHeight,   ///< Input: image height
                        const int nImageWidthStep,
                        const double*  pdH,        ///< Input: 3x3 homography matrix in row-major
                        const float fAlpha, ///< Multiplicative part of the affine illumination model
                        const float fBeta, ///< Additive part of the affine illumination model
                        unsigned char* pWarpedPatch,     ///< Input/Output: memory allocated by the user that will be used to store the result of the warping
                        const int nPatchWidth,     ///< Input: warped patch width
                        const int nPatchHeight,    ///< Input: warped patch height
                        const int nPatchWidthStep, 
                        MaskFunctorT& maskFctr ///< Input/Output: functor that can be used to store the bytes to ignore (out-of-image)
                        ) 
{
    const int nPatchPadding = nPatchWidthStep - nPatchWidth;
    const int nWM = nImageWidth - 1;
    const int nHM = nImageHeight - 1;

    float pH[9];
    for( int ii=0; ii < 9; ii++ ) {
        pH[ii] = float( pdH[ii] );
    }
 
    float fXc,fYc,fZc;
    float fPartXc,fPartYc,fPartZc;

    float fH6m, fH0m, fH3m;
    //
    // dColRef,nRowRef: values taken in the reference image
    // fXc,fYc,fZc : values taken in the current image
    // 
    fPartXc = pH[2];
    fPartYc = pH[5];
    fPartZc = pH[8];    
    for( int nRowRef=0; nRowRef<nPatchHeight; nRowRef++ ) {
        fH6m = fPartZc; fH0m = fPartXc; fH3m = fPartYc;
        for( int nColRef=0; nColRef<nPatchWidth; nColRef++ ) {
            fZc = 1/fH6m;
            fXc = fH0m*fZc;
            fYc = fH3m*fZc;

            fH6m += pH[6];
            fH0m += pH[0];
            fH3m += pH[3];
            
            BilinearInterpolationIllum( nWM, nHM, nImageWidthStep,
                                        pImage, fXc, fYc, fAlpha, fBeta,
                                        pWarpedPatch, maskFctr
                                        );
            pWarpedPatch++;
            //pWarpedPatchMask++;
            maskFctr.fwd();
        }
        for( int kk = 0; kk < nPatchPadding; kk++ ) {
            pWarpedPatch++;
        }
        fPartXc += pH[1];
        fPartYc += pH[4];
        fPartZc += pH[7];    
    }
    maskFctr.end();
}
 
////////////////////////////////////////////////////////////////////////////////
template< class MaskFunctorT >
void CTrack::WarpTranslation( const unsigned char* pImage, ///< Input: image input
                              const int nImageWidth,    ///< Input: image width
                              const int nImageHeight,   ///< Input: image height
                              float dfX, ///< Input: offset in X
                              float dfY, ///< Input: offset in Y
                              unsigned char* pWarpedPatch,     ///< Input/Output: memory allocated by the user that will be used to store the result of the warping
                              const int nPatchWidth,     ///< Input: warped patch width
                              const int nPatchHeight,    ///< Input: warped patch height
                              MaskFunctorT& maskFctr ///< Input/Output: memory allocated by the user that will contain the bytes to ignore (out-of-image)
                              ) 
{ 
    WarpTranslation( pImage, nImageWidth, nImageHeight, nImageWidth, dfX, dfY,
                     pWarpedPatch, nPatchWidth, nPatchHeight, nPatchWidth, maskFctr );
}

////////////////////////////////////////////////////////////////////////////////
void CTrack::WarpTranslation( const unsigned char* pImage, ///< Input: image input
                              const int nImageWidth,    ///< Input: image width
                              const int nImageHeight,   ///< Input: image height
                              float dfX, ///< Input: offset in X
                              float dfY, ///< Input: offset in Y
                              unsigned char* pWarpedPatch,     ///< Input/Output: memory allocated by the user that will be used to store the result of the warping
                              const int nPatchWidth,     ///< Input: warped patch width
                              const int nPatchHeight    ///< Input: warped patch height
                              )
{
    NoMask noMask;
    WarpTranslation( pImage, nImageWidth, nImageHeight, dfX, dfY,
                     pWarpedPatch, nPatchWidth, nPatchHeight, noMask );
}

////////////////////////////////////////////////////////////////////////////////
template< class MaskFunctorT >
void CTrack::WarpTranslation( const unsigned char* pImage, ///< Input: image input
                              const int nImageWidth,    ///< Input: image width
                              const int nImageHeight,   ///< Input: image height
                              const int nImageWidthStep,
                              float dfX, ///< Input: offset in X
                              float dfY, ///< Input: offset in Y
                              unsigned char* pWarpedPatch,     ///< Input/Output: memory allocated by the user that will be used to store the result of the warping
                              const int nPatchWidth,     ///< Input: warped patch width
                              const int nPatchHeight,    ///< Input: warped patch height
                              const int nPatchWidthStep, 
                              MaskFunctorT& maskFctr
                              ) 
{
    const int nPatchPadding = nPatchWidthStep - nPatchWidth;
    const int nWM = nImageWidth - 1;
    const int nHM = nImageHeight - 1;

    float fColRef = 0.;

    //
    // fColRef,nRowRef: values taken in the reference image
    // fXc,fYc: values taken in the current image
    // 
    float fYc = dfY;
    for( int nRowRef=0; nRowRef<nPatchHeight; nRowRef++, fYc++ ) {
        fColRef = dfX;
        for( int nColRef=0; nColRef<nPatchWidth; nColRef++, fColRef++ ) {
            BilinearInterpolation( nWM, nHM,  nImageWidthStep,
                                   pImage, fColRef, fYc, 
                                   pWarpedPatch, maskFctr
                                   );

            pWarpedPatch++;
            //pWarpedPatchMask++;
            maskFctr.fwd();
        }
        for( int kk = 0; kk < nPatchPadding; kk++ ) {
            pWarpedPatch++;
        }
    }
    maskFctr.end();
}

////////////////////////////////////////////////////////////////////////////////
template< class MaskFunctorT >
void CTrack::WarpTranslationIllum( const unsigned char* pImage, ///< Input: image input
                                   const int nImageWidth,    ///< Input: image width
                                   const int nImageHeight,   ///< Input: image height
                                   const int nImageWidthStep,
                                   float dfX, ///< Input: offset in X
                                   float dfY, ///< Input: offset in Y
                                   float fAlpha, float fBeta,
                                   unsigned char* pWarpedPatch,     ///< Input/Output: memory allocated by the user that will be used to store the result of the warping
                                   const int nPatchWidth,     ///< Input: warped patch width
                                   const int nPatchHeight,    ///< Input: warped patch height
                                   const int nPatchWidthStep, 
                                   MaskFunctorT& maskFctr
                                   ) 
{
    const int nPatchPadding = nPatchWidthStep - nPatchWidth;
    const int nWM = nImageWidth - 1;
    const int nHM = nImageHeight - 1;

    float fColRef = 0.;

    //
    // fColRef,nRowRef: values taken in the reference image
    // fXc,fYc: values taken in the current image
    // 
    float fYc = dfY;
    for( int nRowRef=0; nRowRef<nPatchHeight; nRowRef++, fYc++ ) {
        fColRef = dfX;
        for( int nColRef=0; nColRef<nPatchWidth; nColRef++, fColRef++ ) {
            BilinearInterpolationIllum( nWM, nHM,  nImageWidthStep,
                                        pImage, fColRef, fYc, 
                                        fAlpha, fBeta,
                                        pWarpedPatch, maskFctr
                                        );
            pWarpedPatch++;
            //pWarpedPatchMask++;
            maskFctr.fwd();
        }
        for( int kk = 0; kk < nPatchPadding; kk++ ) {
            pWarpedPatch++;
        }
    }
    maskFctr.end();
}

////////////////////////////////////////////////////////////////////////////////
void CTrack::WarpTranslation( const unsigned char* pImage, ///< Input: image input
                              const int nImageWidth,    ///< Input: image width
                              const int nImageHeight,   ///< Input: image height
                              const int nImageWidthStep,
                              float dfX, ///< Input: offset in X
                              float dfY, ///< Input: offset in Y
                              unsigned char* pWarpedPatch,     ///< Input/Output: memory allocated by the user that will be used to store the result of the warping
                              const int nPatchWidth,     ///< Input: warped patch width
                              const int nPatchHeight,    ///< Input: warped patch height
                              const int nPatchWidthStep
                              )
{
    NoMask noMask;
    WarpTranslation( pImage, nImageWidth, nImageHeight, nImageWidthStep, dfX, dfY,
                     pWarpedPatch, nPatchWidth, nPatchHeight, nPatchWidthStep, noMask );
}

////////////////////////////////////////////////////////////////////////////////
bool CTrack::HalveImage( const unsigned char *pInputData, ///< Input: input image
                         const int nWidth,                ///< Input: image width
                         const int nHeight,               ///< Input: image height
                         unsigned char *pOutputData       ///< Ouput: decimated image
                         )
{
    return HalveImage( pInputData, nWidth, nHeight, nWidth,
                       pOutputData, nWidth );
}

////////////////////////////////////////////////////////////////////////////////
bool CTrack::HalveImage( const unsigned char *pInputData, ///< Input: input image
                         const int nWidth,                ///< Input: image width
                         const int nHeight,               ///< Input: image height
                         const int nWidthStep,            ///< Input: 
                         unsigned char *pOutputData,      ///< Output: decimated image
                         const int nOutputWidthStep
                         )
{
    const int nOutputPadding = nOutputWidthStep - nWidth/2;

    int nRowMnWidthStep, nRowP1MnWidthStep;

    for( int nRow = 0; nRow < nHeight-1; nRow+=2 ) {
        nRowMnWidthStep   = nRow * nWidthStep;
        nRowP1MnWidthStep = (nRow+1) * nWidthStep;
        for( int nCol = 0; nCol < nWidth-1; nCol+=2 ) {
            *pOutputData++ = char( ( (int)pInputData[ nRowMnWidthStep + nCol ] +
                        (int)pInputData[ nRowMnWidthStep + nCol + 1 ] +
                        (int)pInputData[ nRowP1MnWidthStep + nCol ] +
                        (int)pInputData[ nRowP1MnWidthStep + nCol + 1 ] )/4 );
        }
        for( int kk = 0; kk < nOutputPadding; kk++ ) {
            pOutputData++;
        }
    }

    return true;
}

////////////////////////////////////////////////////////////////////////////////
void CTrack::Gradient( unsigned char* pImage, ///< Input:
                       const int nImageWidth,       ///< Input:
                       const int nImageHeight,      ///< Input:     
                       float* pGradX,               ///< Output:
                       float* pGradY                ///< Output:
                       )
{
    CTrack::Gradient( pImage, nImageWidth, nImageHeight, 
                      nImageWidth, pGradX, pGradY );
}

////////////////////////////////////////////////////////////////////////////////
void CTrack::Gradient( unsigned char* pImage,  ///< Input:
                       const int nImageWidth,  ///< Input:
                       const int nImageHeight, ///< Input:  
                       const int nImageWidthStep,
                       float* pGradX,          ///< Output:
                       float* pGradY           ///< Output:
                       )
{
    const int nImageWidthM1  = nImageWidth - 1;
    const int nImageHeightM1 = nImageHeight - 1;

    unsigned char* pRow,*pBottomRow,*pTopRow;

    pRow         = pImage; 
    pBottomRow   = pImage + nImageWidthStep;
    float* pRowX = pGradX;
    float* pRowY = pGradY;

    // Work on the first row
    pRowX[0] = pRow[1] - pRow[0];
    pRowY[0] = pBottomRow[0] - pRow[0]; 
    for( int nCol = 1; nCol < nImageWidthM1; ++nCol ) {
        pRowX[nCol] = (pRow[nCol+1] - pRow[nCol-1])/2.;
        pRowY[nCol] = pBottomRow[nCol] - pRow[nCol];
    }
    pRowX[nImageWidthM1] = pRow[nImageWidthM1] - pRow[nImageWidthM1-1];
    pRowY[nImageWidthM1] = pBottomRow[nImageWidthM1] - pRow[nImageWidthM1]; 

    pRow       = pImage + nImageWidthStep; 
    pBottomRow = pImage + 2*nImageWidthStep; 
    pTopRow    = pImage;
    pRowX      = pGradX + nImageWidth;
    pRowY      = pGradY + nImageWidth;

    // Work from the second to the "last-1" row
    for( int nRow = 1; nRow < nImageHeightM1; ++nRow ) {
        // First column
        *pRowX++ = pRow[1] - pRow[0];
        *pRowY++ = (pBottomRow[0] - pTopRow[0])/2.;
    
        for( int nCol = 1; nCol < nImageWidthM1; ++nCol ) {
            *pRowX++ = (pRow[nCol+1] - pRow[nCol-1])/2.;      
            *pRowY++ = (pBottomRow[nCol] - pTopRow[nCol])/2.;
        }

        // Last column
        *pRowX++ = pRow[nImageWidthM1] - pRow[nImageWidthM1-1];
        *pRowY++ = (pBottomRow[nImageWidthM1] - pTopRow[nImageWidthM1])/2.;

        // Move to next rows
        pRow       += nImageWidthStep; 
        pBottomRow += nImageWidthStep; 
        pTopRow    += nImageWidthStep;
    }

    // Last row
    pTopRow  = pImage + ( nImageHeightM1 - 1 ) * nImageWidthStep;
    pRow     = pImage + nImageHeightM1 * nImageWidthStep; 
    pRowX    = pGradX + nImageHeightM1 * nImageWidth;
    pRowY    = pGradY + nImageHeightM1 * nImageWidth;
    pRowX[0] = pRow[1] - pRow[0];  
    pRowY[0] = pRow[0] - pTopRow[0];
  
    for( int nCol = 1; nCol < nImageWidthM1; ++nCol ) {
        pRowX[nCol] = (pRow[nCol+1] - pRow[nCol-1])/2.;
        pRowY[nCol] = pRow[nCol] - pTopRow[nCol];
    }
    pRowX[nImageWidthM1] = pRow[nImageWidthM1] - pRow[nImageWidthM1-1];
    pRowY[nImageWidthM1] = pRow[nImageWidthM1] - pTopRow[nImageWidthM1];
}

#endif
