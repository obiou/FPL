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

#ifndef CTRACK_JACOBIANS_H

#define CTRACK_JACOBIANS_H

///
/// Contains low-level call for computing the Jacobians.
///
/// The functions have three levels:
/// - Higher level call eg. ESMTranslationJacobian
/// - This calls a generic templated function ESMJacobian of ESMJacobianBlur
/// - The templated function takes the computed Jacobian values
/// (- Internally, the code update of the Hessian is then done by a recursive template)
///

#define ESM_TRANSL       0
#define ESM_TRANSL_ILLUM 1
#define ESM_SE2          2
#define ESM_SE2_ILLUM    3
#define ESM_AFFINE       4
#define ESM_AFFINE_ILLUM 5
#define ESM_HOMOG        6
#define ESM_HOMOG_ILLUM  7

#define DOF_TRANSL         2
#define DOF_TRANSL_ILLUM   4
#define DOF_SE2            3
#define DOF_SE2_ILLUM      5
#define DOF_AFFINE         6
#define DOF_AFFINE_ILLUM   8
#define DOF_HOMOG          8
#define DOF_HOMOG_ILLUM   10

#define DOF_HOMOG_BLUR_MAGN    9
#define DOF_HOMOG_ILLUM_BLUR_MAGN   11

// Differentiation with respect to
// the Lie generators
#define JH1(x,y,dx,dy) dx
#define JH2(x,y,dx,dy) dy
#define JH3(x,y,dx,dy) (dx*y - dy*x)
#define JH4(x,y,dx,dy) (dx*x + dy*y)
#define JH5(x,y,dx,dy) (dx*x - dy*y)
#define JH6(x,y,dx,dy) dx*y
#define JH7(x,y,dx,dy) (-dx*x*x - dy*y*x) // do not factorise (the compiler does it better!)
#define JH8(x,y,dx,dy) (-dx*x*y - dy*y*y)

// Homography Jacobians for ESM
#define JK1  JH1(xf,yf,dx,dy)
#define JK2  JH2(xf,yf,dx,dy)
#define JK3  JH3(xf,yf,dx,dy)
#define JK4  JH4(xf,yf,dx,dy)
#define JK5  JH5(xf,yf,dx,dy)
#define JK6  JH6(xf,yf,dx,dy)
#define JK7  JH7(xf,yf,dx,dy)
#define JK8  JH8(xf,yf,dx,dy)

// Blur Jacobians
// Homography blur Jacobian
#define JK1b dLambda*JH1(xf,yf,dxr,dyr)
#define JK2b dLambda*JH2(xf,yf,dxr,dyr)
#define JK3b dLambda*JH3(xf,yf,dxr,dyr)
#define JK4b dLambda*JH4(xf,yf,dxr,dyr)
#define JK5b dLambda*JH5(xf,yf,dxr,dyr)
#define JK6b dLambda*JH6(xf,yf,dxr,dyr)
#define JK7b dLambda*JH7(xf,yf,dxr,dyr)
#define JK8b dLambda*JH8(xf,yf,dxr,dyr)

// Blur magnitude Jacobian
#define Jlambda (-dxr*(logHincr[0]*xf+logHincr[1]*yf+logHincr[2])   \
                 - dyr*(logHincr[3]*xf+logHincr[4]*yf+logHincr[5]) \
                 + (xf*dxr+yf*dyr)*(logHincr[6]*xf+logHincr[7]*yf+logHincr[8]))

// Final homography Jacobian (sum)
#if 1
// Expanded version
#  define JK1s (JK1-JK1b)
#  define JK2s (JK2-JK2b)
#  define JK3s (JK3-JK3b)
#  define JK4s (JK4-JK4b)
#  define JK5s (JK5-JK5b)
#  define JK6s (JK6-JK6b)
#  define JK7s (JK7-JK7b)
#  define JK8s (JK8-JK8b)
#else
// Factorised version
#  define JK1s (JH1(xf,yf,(dx-dLambda*dxr),(dy-dLambda*dyr)))
#  define JK2s (JH2(xf,yf,(dx-dLambda*dxr),(dy-dLambda*dyr)))
#  define JK3s (JH3(xf,yf,(dx-dLambda*dxr),(dy-dLambda*dyr)))
#  define JK4s (JH4(xf,yf,(dx-dLambda*dxr),(dy-dLambda*dyr)))
#  define JK5s (JH5(xf,yf,(dx-dLambda*dxr),(dy-dLambda*dyr)))
#  define JK6s (JH6(xf,yf,(dx-dLambda*dxr),(dy-dLambda*dyr)))
#  define JK7s (JH7(xf,yf,(dx-dLambda*dxr),(dy-dLambda*dyr)))
#  define JK8s (JH8(xf,yf,(dx-dLambda*dxr),(dy-dLambda*dyr)))
#endif

inline int DOF_T( const int DOF ) { return DOF*(DOF+1)/2; }

////////////////////////////////////////////////////////////////////////////////
namespace CTrack {

    ////////////////////////////////////////////////////////////////////////////
    class LineJacobianTranslation {
    public:
        const static int DOF = 2;
        void operator()( double* JTmp,
                         const double dx, const double dy,
                         const double, const double,
                         const unsigned char,
                         const double, const double,
                         const double*, const double
                     ) {
            JTmp[0] = JK1;
            JTmp[1] = JK2;
        }
    };

    ////////////////////////////////////////////////////////////////////////////
    class LineJacobianTranslationIllumination {
    public:
        const static int DOF = 4;
        void operator()( double* JTmp,
                         const double dx, const double dy,
                         const double, const double,
                         const unsigned char uCurIntensity,
                         const double, const double,
                         const double*, const double
                     ) {
            JTmp[0] = JK1;
            JTmp[1] = JK2;  
            JTmp[2] = 2*uCurIntensity; // '2*' To compensate for the div by two at the end.
            JTmp[3] = 2;               // '2' To compensate for the div by two at the end. 
  
        }
    };

    ////////////////////////////////////////////////////////////////////////////
    class LineJacobianSE2 {
    public:
        const static int DOF = 3;
        void operator()( double* JTmp,
                         const double dx, const double dy,
                         const double xf, const double yf,
                         const unsigned char,
                         const double, const double,
                         const double*, const double
                         ) {
            JTmp[0] = JK1;
            JTmp[1] = JK2;
            JTmp[2] = JK3;
        }
    };

    /// ////////////////////////////////////////////////////////////////////////////
    class LineJacobianSE2Illumination {
    public:
        const static int DOF = 5;
        void operator()( double* JTmp,
                         const double dx, const double dy,
                         const double xf, const double yf,
                         const unsigned char uCurIntensity,
                         const double, const double,
                         const double*, const double
                     ) {
            JTmp[0] = JK1;
            JTmp[1] = JK2;
            JTmp[2] = JK3;  
            JTmp[3] = 2*uCurIntensity; // '2*' To compensate for the div by two at the end.
            JTmp[4] = 2;               // '2' To compensate for the div by two at the end. 
        }
    };

    /////////////////////////////////////////////////////////////////////////
    class LineJacobianAffine {
    public:
        const static int DOF = 6;
        void operator()( double* JTmp,
                         const double dx, const double dy,
                         const double xf, const double yf,
                         const unsigned char,
                         const double, const double,
                         const double*, const double
                     ) {
            JTmp[0] = JK1;
            JTmp[1] = JK2;
            JTmp[2] = JK3;
            JTmp[3] = JK4;
            JTmp[4] = JK5;
            JTmp[5] = JK6;
        }
    };

    ////////////////////////////////////////////////////////////////////////////
    class LineJacobianAffineIllumination {
    public:
        const static int DOF = 8;
        void operator()( double* JTmp,
                         const double dx, const double dy,
                         const double xf, const double yf,
                         const unsigned char uCurIntensity,
                         const double, const double,
                         const double*, const double
                     ) {
            JTmp[0] = JK1;
            JTmp[1] = JK2;
            JTmp[2] = JK3;
            JTmp[3] = JK4;
            JTmp[4] = JK5;
            JTmp[5] = JK6;
            JTmp[6] = 2*uCurIntensity; // '2*' To compensate for the div by two at the end.
            JTmp[7] = 2;               // '2' To compensate for the div by two at the end. 
        }
    };

    ////////////////////////////////////////////////////////////////////////////
    class LineJacobianHomography {
    public:
        const static int DOF = 8;
        void operator()( double* JTmp,
                         const double dx, const double dy,
                         const double xf, const double yf,
                         const unsigned char,
                         const double, const double,
                         const double*, const double
                     ) {
            JTmp[0] = JK1;
            JTmp[1] = JK2;
            JTmp[2] = JK3;
            JTmp[3] = JK4;
            JTmp[4] = JK5;
            JTmp[5] = JK6;
            JTmp[6] = JK7;
            JTmp[7] = JK8;
        }
    };

    ////////////////////////////////////////////////////////////////////////////
    class LineJacobianHomographyIllumination {
    public:
        const static int DOF = 10;
        void operator()( double* JTmp,
                         const double dx, const double dy,
                         const double xf, const double yf,
                         const unsigned char uCurIntensity,
                         const double, const double,
                         const double*, const double
                     ) {
            JTmp[0] = JK1;
            JTmp[1] = JK2;
            JTmp[2] = JK3;
            JTmp[3] = JK4;
            JTmp[4] = JK5;
            JTmp[5] = JK6;
            JTmp[6] = JK7;
            JTmp[7] = JK8;
            JTmp[8] = 2*uCurIntensity; // '2*' To compensate for the div by two at the end.
            JTmp[9] = 2;               // '2' To compensate for the div by two at the end.
        }
    };

    ////////////////////////////////////////////////////////////////////////////
    class LineJacobianHomographyBlurMagn {
    public:
        const static int DOF = 9;
        void operator()( double* JTmp,
                         const double dx, const double dy,
                         const double xf, const double yf,
                         const unsigned char,
                         const double dxr, const double dyr,
                         const double* logHincr, // log(HEst) *without* the multiplication by the magnitude 'lambda'
                         const double dLambda  // blur magnitude (percentage of time the shutter is open)
                         ) {
            JTmp[0] = JK1s;
            JTmp[1] = JK2s;
            JTmp[2] = JK3s;
            JTmp[3] = JK4s;
            JTmp[4] = JK5s;
            JTmp[5] = JK6s;
            JTmp[6] = JK7s;
            JTmp[7] = JK8s;
            JTmp[8] = Jlambda;
        }
    };

    ////////////////////////////////////////////////////////////////////////////
    class LineJacobianHomographyBlurMagnIllumination {
    public:
        const static int DOF = 11;
        void operator()( double* JTmp,
                         const double dx, const double dy,
                         const double xf, const double yf,
                         const unsigned char uCurIntensity,
                         const double dxr, const double dyr,
                         const double* logHincr, // log(HEst) *without* the multiplication by the magnitude 'lambda'
                         const double dLambda  // blur magnitude (percentage of time the shutter is open)
                         ) {
            JTmp[0]  = JK1s;
            JTmp[1]  = JK2s;
            JTmp[2]  = JK3s;
            JTmp[3]  = JK4s;
            JTmp[4]  = JK5s;
            JTmp[5]  = JK6s;
            JTmp[6]  = JK7s;
            JTmp[7]  = JK8s;
            JTmp[8]  = Jlambda; 
            JTmp[9]  = uCurIntensity; // We divide by 2 when computing the image Jacobians
            JTmp[10] = 1;
       }
    };

    ////////////////////////////////////////////////////////////////////////////
    /// Main function used to compute Jacobians.
    /// It is templated on the number of degrees of freedom (DOF) or
    /// number of parameters being estimated and a function that computes
    /// the Jacobian for a given 'row' or pixel function.
    template<class LineJacobianFunc>
        void ESMJacobian( unsigned char* pRefPatch, 
                          float* pRefPatchGradX,
                          float* pRefPatchGradY,
                          unsigned char* pCurPatch,
                          float* pCurPatchGradX,
                          float* pCurPatchGradY,
                          const int nPatchWidth, 
                          const int nPatchHeight,
                          const int nPatchWidthStep,
                          double* JtJ, ///< Output: triangular part of J'*J, of size DOF_TRANSL_T
                          double* JtE, ///< Output: J'*(current_pixel_value - referenc_pixel_value), of size DOF_TRANSL
                          double* dRMS ///< Output: root mean square error (useful for checking decreasing error)
                          );
    
    ////////////////////////////////////////////////////////////////////////////
    /// Main function used to compute Jacobians.
    /// It is templated on the number of degrees of freedom (DOF) or
    /// number of parameters being estimated and a function that computes
    /// the Jacobian for a given 'row' or pixel function.
    template<class LineJacobianFunc>
        void ESMJacobianBlur( unsigned char* pBlurredRefPatch, 
                              float* pBlurredRefPatchGradX,
                              float* pBlurredRefPatchGradY, 
                              float* pTimeWeightedBlurredRefPatchGradX,
                              ///< Input: image gradient of the reference patch blurred and weighted by time in x
                              float* pTimeWeightedBlurredRefPatchGradY,
                              ///< Input: image gradient of the reference patch blurred and weig
                              unsigned char* pCurPatch,
                              float* pCurPatchGradX,
                              float* pCurPatchGradY,
                              const int nPatchWidth, 
                              const int nPatchHeight,
                              const int nPatchWidthStep,   
                              const double* logHincr, // log(HEst) *without* the multiplication by the magnitude 'lambda'
                              const double dLambda, // blur magnitude (percentage of time the shutter is open)
                              double* JtJ, ///< Output: triangular part of J'*J, of size DOF_TRANSL_T
                              double* JtE, ///< Output: J'*(current_pixel_value - referenc_pixel_value), of size DOF_TRANSL
                              double* dRMS ///< Output: root mean square error (useful for checking decreasing error)
                              );
}

#endif
