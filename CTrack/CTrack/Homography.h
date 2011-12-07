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

#ifndef CTRACK_HOMOGRAPHY_H
#define CTRACK_HOMOGRAPHY_H

#include <iostream>
#include <utility>
#include <vector>

#include <eigen3/Eigen/Core>

namespace CTrack {
    const double dTOL = 1e-6;

    ////////////////////////////////////////////////////////////////////////////
    float randf();
    
    ////////////////////////////////////////////////////////////////////////////
    /// Compute random number from Gausssian distribution, returns two
    /// values at once (Box-Muller transform)
    void rand_gaussf( float fMean, float fStd, float& y1, float& y2 );

    ////////////////////////////////////////////////////////////////////////////
    /// Compute random number from Gausssian distribution, returns two
    /// values at once (Box-Muller transform)
    void rand_gaussf( float& y1, float& y2 );

    ////////////////////////////////////////////////////////////////////////////
    bool SolveNormalEquations( const int nDOF, double* JtJ, double* JtE,
                               Eigen::VectorXd& vESMUpdate );

    ////////////////////////////////////////////////////////////////////////////
    Eigen::MatrixXd MakeFullSymmetricMatrix( const int nDOF, double* JtJ );

    ////////////////////////////////////////////////////////////////////////////
    /// Given vESMUpdate, this function will compute
    /// sum_i(vESMUpdate(i)*G_i) with {G_i,i=1..8} a basis (generators) of sl3.
    Eigen::Matrix3d MakeSumGenerators( const Eigen::VectorXd& vESMUpdate );

    ////////////////////////////////////////////////////////////////////////////
    Eigen::Matrix3d expm_series( const Eigen::Matrix3d& mM );

    ////////////////////////////////////////////////////////////////////////////
    // Cheap expm computation (not exact...)
    Eigen::Matrix3d expm( const Eigen::Matrix3d& mM );

    ////////////////////////////////////////////////////////////////////////////
    /// This function computes the SL3 matrix from a vector of sl3.
    /// i.e. exp( sum_i( vESMUpdate(i)*G_i )
    /// If vESMUpdate does not have 8 values, only the
    /// vESMUpdate.size() first generators will be used.
    Eigen::Matrix3d ComputeSL3Update( const Eigen::VectorXd& vESMUpdate );

    ////////////////////////////////////////////////////////////////////////////
    class Homography;
    CTrack::Homography GenerateScaled( double dScale );

    ////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////
    class Homography {
    public:
        ////////////////////////////////////////////////////////////////////////
        Homography() {
            m_mH = Eigen::Matrix3d::Identity();
#ifndef EIGEN_DEFAULT_TO_ROW_MAJOR
            m_mHt = Eigen::Matrix3d::Identity();
#endif
        }

        ////////////////////////////////////////////////////////////////////////
        Homography( const Eigen::Matrix3d& mH ) {
            m_mH = mH;
        }

        ////////////////////////////////////////////////////////////////////////
        Homography( std::vector<double>& vLieValues ) {
            Eigen::Map<Eigen::VectorXd> vESMUpdate( &vLieValues[0], vLieValues.size() );
            m_mH = ComputeSL3Update( vESMUpdate );
        }

        ////////////////////////////////////////////////////////////////////////
        ~Homography() {
        }

        ////////////////////////////////////////////////////////////////////////////
        /// Writes a homography to a flow
        inline friend std::ostream& operator<< ( std::ostream& stream, 
                                          const Homography& H ) {
            stream << H.m_mH << std::endl;
            return stream;
        }
        
        ////////////////////////////////////////////////////////////////////////////
        /// Reads a homography from a flow
        inline friend std::istream& operator>> ( std::istream& stream, 
                                          Homography& H ) {
            double h1,h2,h3,h4,h5,h6,h7,h8,h9;
            stream >> h1 >> h2 >> h3 >> h4 >> h5 >> h6 >> h7>> h8 >> h9;
            H.m_mH << h1, h2, h3, h4, h5, h6, h7, h8, h9;
            return stream;
        }

        ////////////////////////////////////////////////////////////////////////
        void id() {
            m_mH = Eigen::Matrix3d::Identity();
        }

        ////////////////////////////////////////////////////////////////////////
        void zero() {
            m_mH = Eigen::Matrix3d::Zero();
        }
        
        ////////////////////////////////////////////////////////////////////////
        void mult( const Homography& H ) {
            m_mH *= H.m_mH;
        }

        ////////////////////////////////////////////////////////////////////////
        void mult( const double dScalar ) {
            m_mH *= dScalar;
        }

        ////////////////////////////////////////////////////////////////////////
        /// Axis-angle update
        void rot( const double d1, const double d2, const double d3 ) {
            Eigen::Matrix<double,3,1> vESMUpdate;
            vESMUpdate[0] = d1;
            vESMUpdate[1] = d2;
            vESMUpdate[2] = d3;
            m_mH *= ComputeSL3Update( vESMUpdate );
        }

        ////////////////////////////////////////////////////////////////////////
        void rot( const double dAngle ) {
            Eigen::Matrix<double,3,1> vESMUpdate;
            vESMUpdate[0] = 0;
            vESMUpdate[1] = 0;
            vESMUpdate[2] = dAngle/180.*3.14;
            m_mH *= ComputeSL3Update( vESMUpdate );
        }

        ////////////////////////////////////////////////////////////////////////
        /// Replace homography H with:  H(cx/2,cy/2)*H*H(-cx/2,-cy/2)
        void center( const double dCx, const double dCy ) {
            Eigen::Matrix3d mHC = Eigen::Matrix3d::Identity();
            mHC( 0, 2 ) = dCx/2;
            mHC( 1, 2 ) = dCy/2;
            Eigen::Matrix3d mHCInv = Eigen::Matrix3d::Identity();
            mHCInv( 0, 2 ) = -dCx/2;
            mHCInv( 1, 2 ) = -dCy/2;

            m_mH = mHC * m_mH * mHCInv;
        }

        ////////////////////////////////////////////////////////////////////////
        /// Generate a random matrix
        /// WARNING: if the generators are changes, this function has to be changed
        void rand( const double dMeanTx, const double dStdTx, ///< Input: translation in x
                   const double dMeanTy, const double dStdTy, ///< Input: translation in y
                   const double dMeanRz, const double dStdRz, ///< Input: inplane rotation (rotation around z)
                   const double dMeanS,  const double dStdS,  ///< Input: exp-scale
                   const double dMeanA,  const double dStdA,  ///< Input: exp-aspect ratio
                   const double dMeanSh, const double dStdSh, ///< Input: shear
                   const double dMeanRy, const double dStdRy, ///< Input: part of out-of-plane rotation around y
                   const double dMeanRx, const double dStdRx  ///< Input: part of out-of-plane rotation around y
                  ) {
            Eigen::Matrix<double,8,1> vESMUpdate;

            float dTmp, dSample;
            // Draw samples and fill SL3 update vector (i.e. lie algebra term)
            rand_gaussf( dMeanTx, dStdTx, dSample, dTmp );
            vESMUpdate[0] = dSample;
            rand_gaussf( dMeanTy, dStdTy, dSample, dTmp );
            vESMUpdate[1] = dSample;
            rand_gaussf( dMeanRz, dStdRz, dSample, dTmp );
            vESMUpdate[2] = dSample;
            rand_gaussf( dMeanS, dStdS, dSample, dTmp );
            vESMUpdate[3] = dSample;
            rand_gaussf( dMeanA, dStdA, dSample, dTmp );
            vESMUpdate[4] = dSample;
            rand_gaussf( dMeanSh, dStdSh, dSample, dTmp );
            vESMUpdate[5] = dSample;
            rand_gaussf( dMeanRy, dStdRy, dSample, dTmp );
            vESMUpdate[6] = dSample;            
            rand_gaussf( dMeanRx, dStdRx, dSample, dTmp );
            vESMUpdate[7] = dSample;            

            id();
            m_mH *= ComputeSL3Update( vESMUpdate );
        }

        ////////////////////////////////////////////////////////////////////////
        /// Matrix will become inv(Hs)*H*Hs with Hs a scaling matrix
        /// defined by dScale.
        CTrack::Homography scale( double dScale ) { 
            Eigen::Matrix3d MScaleDown = Eigen::Matrix3d::Identity();
            Eigen::Matrix3d MScaleUp = Eigen::Matrix3d::Identity();
            MScaleDown(0,0) = 1./dScale;
            MScaleDown(1,1) = 1./dScale;
            MScaleUp(0,0)   = dScale;
            MScaleUp(1,1)   = dScale;
            m_mH = MScaleDown*m_mH*MScaleUp;
            return *this; // return a copy
        }

        ////////////////////////////////////////////////////////////////////////
        void inv();

        ////////////////////////////////////////////////////////////////////////
        void expm() {
            m_mH = CTrack::expm( m_mH );
        }

        ////////////////////////////////////////////////////////////////////////
        void logm();

        ////////////////////////////////////////////////////////////////////////
        template<class T1, class T2>
            std::pair<T2,T2> warp( const std::pair<T1,T1>& pIn ) const {
            Eigen::Vector3d vIn( pIn.first, pIn.second, 1 );
            Eigen::Vector3d vOut = m_mH*vIn;
            return std::pair<T2,T2>( vOut(0)/vOut(2), 
                                     vOut(1)/vOut(2) );
        }

        ////////////////////////////////////////////////////////////////////////
        template<class T1, class T2>
        void warp_polymult( const std::vector<std::pair<T1,T1> >& vPolyIn,
                            std::vector<std::pair<T2,T2> >& vPolyOut
                            ) const {
            vPolyOut.clear();
            for( size_t ii=0; ii<vPolyIn.size(); ii++ ) {
                Eigen::Vector3d vIn( vPolyIn[ii].first, vPolyIn[ii].second, 1 );
                Eigen::Vector3d vOut = m_mH*vIn;
                vPolyOut.push_back( std::pair<T2,T2>( vOut(0)/vOut(2), 
                                                      vOut(1)/vOut(2) ) );
                //std::cout << vPolyOut[ii].first << " " << vPolyOut[ii].second << std::endl;
            }
        }

        ////////////////////////////////////////////////////////////////////////
        void warp_polymult_exp( const std::vector<std::pair<double,double> >& vPolyIn,
                                std::vector<std::pair<double,double> >& vPolyOut
                            ) const {
            vPolyOut.clear();
            for( size_t ii=0; ii<vPolyIn.size(); ii++ ) {
                Eigen::Vector3d vIn( vPolyIn[ii].first, vPolyIn[ii].second, 1 );
                Eigen::Vector3d vOut = CTrack::expm( m_mH )*vIn;
                vPolyOut.push_back( std::pair<double,double>( vOut(0)/vOut(2), 
                                                              vOut(1)/vOut(2) ) );
                //std::cout << vPolyOut[ii].first << " " << vPolyOut[ii].second << std::endl;
            }
        }
      
        ////////////////////////////////////////////////////////////////////////
        void Set( const int nX, const int nY, const double dValue ) {
            m_mH( nX,  nY ) = dValue;
        }

        ////////////////////////////////////////////////////////////////////////
        template<class T>
            void set( T* mH ) {
            for( size_t ii=0; ii<3; ii++ ) {
                for( size_t jj=0; jj<3; jj++ ) {
                    m_mH( ii, jj ) = mH[ 3*ii + jj ];
                }
            }
        }

        ////////////////////////////////////////////////////////////////////////
        double Get( const int nX, const int nY ) const {
            return m_mH( nX,  nY );
        }

        ////////////////////////////////////////////////////////////////////////
        /// Returns a row-major representation of the matrix
        const double* GetRowMajorPtr() {
            //#if EIGEN_DEFAULT_TO_ROW_MAJOR
            if( Eigen::Matrix3d::Base::IsRowMajor ) {
                return const_cast<double*>( m_mH.data() );
            }
            else {
                m_mHt = m_mH.transpose();
                return const_cast<double*>( m_mHt.data() );
            }
        }

        ////////////////////////////////////////////////////////////////////////
        void print() const {
            std::cout << m_mH << std::endl;
        }

        ////////////////////////////////////////////////////////////////////////
        void print( const char * sMsg ) const {
            std::cout << sMsg << std::endl << m_mH << std::endl;
        }

        ////////////////////////////////////////////////////////////////////////
        bool R2Update( double* JtJ,        ///< Input
                       double* JtE,        ///< Input
                       double* dNormUpdate ///< Output
                       ) {

            double dDet = JtJ[0]*JtJ[2] - JtJ[1]*JtJ[1];

            if( dDet>0 && JtJ[0]>=0 && JtJ[2]>=0 ) {
                dDet = -1/dDet;

                double dUpdateX = ( JtJ[2]*JtE[0] - JtJ[1]*JtE[1])*dDet;
                double dUpdateY = (-JtJ[1]*JtE[0] + JtJ[0]*JtE[1])*dDet;

                *dNormUpdate = sqrt( dUpdateX*dUpdateX + dUpdateY*dUpdateY );

                m_mH(0,2) += dUpdateX;
                m_mH(1,2) += dUpdateY;
                return true;
            } 
            else {
                return false;
            }
        }

        ////////////////////////////////////////////////////////////////////////
        bool IsPureTranslation() {
            return std::abs( m_mH(0,0)-1) < dTOL &&
                std::abs( m_mH(1,1)-1) < dTOL &&
                std::abs( m_mH(2,2)-1) < dTOL &&
                std::abs( m_mH(0,1) ) < dTOL &&
                std::abs( m_mH(1,0) ) < dTOL &&
                std::abs( m_mH(2,0) ) < dTOL &&
                std::abs( m_mH(2,1) ) < dTOL;
        }

        ////////////////////////////////////////////////////////////////////////
        bool _GenericSL3Update( const int nDOF,
                                double* JtJ,        ///< Input
                                double* JtE,        ///< Input
                                double* dNormUpdate ) {
            Eigen::VectorXd vESMUpdate;
            bool bSuccess = 
                SolveNormalEquations( nDOF, JtJ, JtE, vESMUpdate );
 
            if( !bSuccess ) {
                return bSuccess;
            }

            *dNormUpdate = vESMUpdate.norm();
           
            m_mH *= ComputeSL3Update( vESMUpdate );
            return bSuccess;
        }

        ////////////////////////////////////////////////////////////////////////
        void SL3Update( int nDOF, double* dUpdate );

        ////////////////////////////////////////////////////////////////////////
        bool SL2R2Update( double* JtJ,        ///< Input
                          double* JtE,        ///< Input
                          double* dNormUpdate ///< Output
                          );

        ////////////////////////////////////////////////////////////////////////
        bool SE2Update( double* JtJ,        ///< Input
                        double* JtE,        ///< Input
                        double* dNormUpdate ///< Output
                        );

        ////////////////////////////////////////////////////////////////////////
        bool SL3Update( double* JtJ,        ///< Input
                        double* JtE,        ///< Input
                        double* dNormUpdate ///< Output
                        );

        ////////////////////////////////////////////////////////////////////////
        bool SL2R2IllumUpdate( double* JtJ,        ///< Input
                               double* JtE,        ///< Input
                               double* dNormUpdate, ///< Output
                               double* dAlpha, ///< Output
                               double* dBeta   ///< Output
                               );

        ////////////////////////////////////////////////////////////////////////
        bool SL3IllumUpdate( double* JtJ,        ///< Input
                             double* JtE,        ///< Input
                             double* dNormUpdate, ///< Output
                             double* dAlpha, ///< Output
                             double* dBeta   ///< Output
                             );

        ////////////////////////////////////////////////////////////////////////
        bool SL3BlurMagnUpdate( double* JtJ,        ///< Input
                                double* JtE,        ///< Input
                                double* dNormUpdate, ///< Output
                                double* dLambda
                                );

        ////////////////////////////////////////////////////////////////////////
        bool SL3BlurMagnIllumUpdate( double* JtJ,        ///< Input
                                     double* JtE,        ///< Input
                                     double* dNormUpdate, ///< Output
                                     double* dLambda,
                                     double* dAlpha,
                                     double* dBeta
                                     );

        ////////////////////////////////////////////////////////////////////////
        Eigen::Matrix3d m_mH;
    private:
        //#ifndef EIGEN_DEFAULT_TO_ROW_MAJOR
        Eigen::Matrix3d m_mHt;
        //#endif
    };
}

#endif
