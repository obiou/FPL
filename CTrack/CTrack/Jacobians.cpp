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

#include <CTrack/Jacobians.h>

#include <cmath>
#include <string.h>

////////////////////////////////////////////////////////////////////////////////
/// AddOuterProd
/// This will compute JtJ += JTmp'*JTmp
///
/// It has been made a recursive template to force the compiler to unroll the loop.
/// It does the same as the simpler 'AddOuterProdSimple'.
/// Pointer arithmetic has been used as it does seem to improve the computation if
/// the pointer to the pointer is used. This is a bit ugly as the user has to take care
/// to store the old value of the beginning of the table JtJ and
/// restore it at the end of the loop (hence the warning).
///
/// WARNING: for optimisation reasons, AddOuterProd has the side
/// effect of MODIFYING the pointer JtJ, remember to store its value
/// if you wish to use it later.
///
////////////////////////////////////////////////////////////////////////////////
template <int DOF, int ROW, int COL>
    class AddOuterProd
{
 public:
    inline static void call( double** JtJ, const double* JTmp ) {
        *(*JtJ)++ += JTmp[DOF-COL]*JTmp[DOF-ROW];
        AddOuterProd<DOF,ROW, COL - 1 >::call( JtJ, JTmp );
    }
};
 
////////////////////////////////////////////////////////////////////////////////
template <int DOF, int ROW>
    class AddOuterProd<DOF,ROW,ROW>
{
 public:
    inline static void call( double** JtJ, const double* JTmp ) {
        *(*JtJ)++ += JTmp[DOF-ROW]*JTmp[DOF-ROW];
        AddOuterProd<DOF,ROW-1, DOF >::call( JtJ, JTmp );
    }
};

////////////////////////////////////////////////////////////////////////////////
template <int DOF>
class AddOuterProd<DOF,1,1>
{
 public:
    inline static void call( double** JtJ, const double* JTmp  ) {
        *(*JtJ) += JTmp[DOF-1]*JTmp[DOF-1];
    }
};

////////////////////////////////////////////////////////////////////////////////
/// To compare with the templated version AddOuterProd
inline void AddOuterProdSimple( const int nDOF, double* JtJ, double* JTmp ) 
{
    double* JTmpRow = JTmp;
    double* JTmpCol = JTmp;
    
    for( int nRow = 0; nRow < nDOF; nRow++ ) {
        JTmpCol = JTmp;
        for( int nCol = 0; nCol <= nRow; nCol++ ) {
            *JtJ++ += *JTmpCol++ * *JTmpRow; 
            }
        JTmpRow++;
        }    
}
    
////////////////////////////////////////////////////////////////////////////////
/// This will compute JtE += JTmp'*dE
/// A recursive templated version as for 'AddOuterProd' did not seem
/// to provide an advantage here.
////////////////////////////////////////////////////////////////////////////////
inline void AddJtE( const int nDOF, double* JtE, double* JTmp, const double dE ) 
{
    for( int nRow = 0; nRow < nDOF; nRow++ ) {
        *JtE++ += *JTmp++ * dE;
    }    
}

////////////////////////////////////////////////////////////////////////////////
namespace CTrack {
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
                          )
    {
        LineJacobianFunc lineJac;
        const int DOF = LineJacobianFunc::DOF;
        const int nPadding = nPatchWidthStep - nPatchWidth;

        const double dummy1=0,dummy2=0,dummy3=0,dummy4=0;

        double dx,dy,xf,yf,dE;
        double JTmp[ DOF ];
        double* JtJ_store = JtJ;
        memset( JtJ, 0, DOF_T( DOF )*sizeof( double ) ); 
        memset( JtE, 0, DOF*sizeof( double ) ); 

        *dRMS = 0;

        yf = 0.;
        for( int y=0; y<nPatchHeight; y++, yf++ ) {
            xf = 0.;
            for( int x=0; x<nPatchWidth; x++, xf++ ) {
                // We do not divide by 2 yet...
                // multiply by 2 at the end
                dx = (double)*pCurPatchGradX++ + *pRefPatchGradX++;
                dy = (double)*pCurPatchGradY++ + *pRefPatchGradY++;
                     
                if( std::isnan( dx ) || std::isnan(dy) ) {
                    pCurPatch++; pRefPatch++;
                    continue;
                }

                // Compute the row 'y' of the Jacobian
                lineJac( JTmp, dx, dy, xf, yf, *pCurPatch,
                         dummy1, dummy2, &dummy3, dummy4 );
      
                // Calculate JtJ (The symmetric matrix is stored as a lower triangular matrix)    
                AddOuterProd<DOF,DOF,DOF>::call( &JtJ, JTmp );

                dE = (double)*pCurPatch++ - *pRefPatch++;

                if( std::isnan(dE) ) {
                    //cout << "******" << endl;
                    //fflush(stdout);
                    // This is in fact only useful
                    // to help the compiler (I'm not sure why)
                    // We shouldn't arrive here, the Jacobian will 
                    // be incorrect if we do...
                    continue;
                }

                AddJtE( DOF, JtE, JTmp, dE );

                *dRMS += dE*dE;
                JtJ = JtJ_store;
            }
            for( int ii=0; ii < nPadding; ii++ ) {
                pCurPatch++;
                pRefPatch++;
            }
        }
        for( int ii=0; ii<DOF_T( DOF ); ii++ ) {
            JtJ[ii] /= 4;
        }
        for( int ii=0; ii<DOF; ii++ ) {
            JtE[ii] /= 2;
        }
        *dRMS = sqrt(*dRMS/(nPatchWidth*nPatchHeight));
    }
    
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
                          )
    {
        LineJacobianFunc lineJac;
        const int DOF = LineJacobianFunc::DOF;
        const int nPadding = nPatchWidthStep - nPatchWidth;

        double dx,dy,xf,yf,dxr,dyr,dE;
        double JTmp[ DOF ];
        double* JtJ_store = JtJ;
        memset( JtJ, 0, DOF_T( DOF )*sizeof( double ) ); 
        memset( JtE, 0, DOF*sizeof( double ) ); 

        *dRMS = 0;

        yf = 0.;
        for( int y=0; y<nPatchHeight; y++, yf++ ) {
            xf = 0.;
            for( int x=0; x<nPatchWidth; x++, xf++ ) {
                // We do not divide by 2 yet...
                // multiply by 2 at the end
                dx  = (double) (*pCurPatchGradX++ + *pBlurredRefPatchGradX++)/2.;
                dy  = (double) (*pCurPatchGradY++ + *pBlurredRefPatchGradY++)/2.;
                dxr = (double) *pTimeWeightedBlurredRefPatchGradX++;
                dyr = (double) *pTimeWeightedBlurredRefPatchGradY++;

                if( std::isnan( dx ) || std::isnan(dy) ) {
                    pCurPatch++; pBlurredRefPatch++;
                    continue;
                }

                // Compute the row 'y' of the Jacobian
                lineJac( JTmp, dx, dy, xf, yf, *pCurPatch,
                         dxr, dyr, logHincr, dLambda );
      
                // Calculate JtJ (The symmetric matrix is stored as a lower triangular matrix)    
                AddOuterProd<DOF,DOF,DOF>::call( &JtJ, JTmp );

                dE = (double)*pCurPatch++ - *pBlurredRefPatch++;

                if( std::isnan(dE) ) {
                    //cout << "******" << endl;
                    //fflush(stdout);
                    // This is in fact only useful
                    // to help the compiler (I'm not sure why)
                    // We shouldn't arrive here, the Jacobian will 
                    // be incorrect if we do...
                    continue;
                }

                AddJtE( DOF, JtE, JTmp, dE );

                *dRMS += dE*dE;
                JtJ = JtJ_store;
            }
            for( int ii=0; ii < nPadding; ii++ ) {
                pCurPatch++;
                pBlurredRefPatch++;
            }
        }
        // In the blur case, the division is done before

        *dRMS = sqrt(*dRMS/(nPatchWidth*nPatchHeight));
    }

    // Instantiate
#define INSTANTIATE_ESM_JACOBIAN( T1 )                                  \
    template void ESMJacobian<T1>( unsigned char*, float*, float*, unsigned char*, \
                                   float*, float*, const int, const int, const int, double*, double*, double* );

#define INSTANTIATE_ESM_JACOBIAN_BLUR( T1 )                             \
    template void ESMJacobianBlur<T1>( unsigned char*, float*, float*, float*, float*, \
    unsigned char*, float*, float*, const int, const int, const int, const double*, \
        const double, double*, double*, double* );

    INSTANTIATE_ESM_JACOBIAN( CTrack::LineJacobianTranslation );
    INSTANTIATE_ESM_JACOBIAN( CTrack::LineJacobianTranslationIllumination );
    INSTANTIATE_ESM_JACOBIAN( CTrack::LineJacobianSE2 );
    INSTANTIATE_ESM_JACOBIAN( CTrack::LineJacobianSE2Illumination );
    INSTANTIATE_ESM_JACOBIAN( CTrack::LineJacobianAffine );
    INSTANTIATE_ESM_JACOBIAN( CTrack::LineJacobianAffineIllumination );
    INSTANTIATE_ESM_JACOBIAN( CTrack::LineJacobianHomography );
    INSTANTIATE_ESM_JACOBIAN( CTrack::LineJacobianHomographyIllumination );

    INSTANTIATE_ESM_JACOBIAN_BLUR( CTrack::LineJacobianHomographyBlurMagn );
    INSTANTIATE_ESM_JACOBIAN_BLUR( CTrack::LineJacobianHomographyBlurMagnIllumination );
}

