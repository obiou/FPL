#ifndef PINHOLE_RAD_TAN_CAMERA_H
#define PINHOLE_RAD_TAN_CAMERA_H

#include <bitset>
#include <iostream>

#include <CCameraModel/ProjectionFunctions.h>

#ifdef CCAMERAMODEL_HAS_OPENCV
#  include <cv.h>
#  include <highgui.h>
#endif

///
/// Pinhole camera with radial and tangential distortion
///
namespace CCameraModel {
    ////////////////////////////////////////////////////////////////////////////
    const int PINHOLE_RADTAN_MAX_NUM_PARAMS = 8;

    ////////////////////////////////////////////////////////////////////////////
    const int FX_INDEX = 0;
    const int FY_INDEX = 1;
    const int CX_INDEX = 2;
    const int CY_INDEX = 3;
    const int K1_INDEX = 4;
    const int K2_INDEX = 5;
    const int P1_INDEX = 6;
    const int P2_INDEX = 7;

    ////////////////////////////////////////////////////////////////////////////
    /// Specific camera: Pinhole model
    FACTORY_OBJECT( PinholeRadTanCamera, CameraModelInterface, "PinholeRadTan" ) {
    public:
        ////////////////////////////////////////////////////////////////////////
    PinholeRadTanCamera() : 
        m_dfx(0), m_dfy(0), m_dcx(0), m_dcy(0),
            m_dk1(0), m_dk2(0), m_dp1(0), m_dp2(0),
            m_nImageWidth( 0 ), m_nImageHeight( 0 ) { 
            m_Settings[FX_INDEX] = 1;
            m_Settings[FY_INDEX] = 1;
            m_Settings[CX_INDEX] = 1;
            m_Settings[CY_INDEX] = 1;
        }

        ////////////////////////////////////////////////////////////////////////
        bool project( const unsigned int nNumPoints,
                      const double* pPoints3D,
                      double* pPoints2D,
                      bool bColMajor
                      ) const {
            return project<double>( nNumPoints, pPoints3D, pPoints2D, bColMajor );
        }

        ////////////////////////////////////////////////////////////////////////
        bool project( const unsigned int nNumPoints,
                      const float* pPoints3D,
                      float* pPoints2D,
                      bool bColMajor
                      ) const {
            return project<float>( nNumPoints, pPoints3D, pPoints2D, bColMajor );
        }

        ////////////////////////////////////////////////////////////////////////
        bool project( const unsigned int nNumPoints,
                      const double* pPoints3D,
                      double* pPoints2D,
                      double* pd2d3d,
                      bool bColMajor
                      ) const {
            return project<double>( nNumPoints, pPoints3D, pPoints2D,
                                    pd2d3d, bColMajor );
        }

        ////////////////////////////////////////////////////////////////////////
        bool project( const unsigned int nNumPoints,
                      const float* pPoints3D,
                      float* pPoints2D,
                      float* pd2d3d,
                      bool bColMajor
                      ) const {
            return project<float>( nNumPoints, pPoints3D, pPoints2D,
                                   pd2d3d, bColMajor );
        }

        ////////////////////////////////////////////////////////////////////////
        bool project( const unsigned int nNumPoints,
                      const double* pPoints3D,
                      double* pPoints2D,
                      double* pd2d3d,
                      double* pd2dParam,
                      bool bColMajor
                      ) const {
            return project<double>( nNumPoints, pPoints3D, pPoints2D,
                                    pd2d3d, pd2dParam, bColMajor );
        }

        ////////////////////////////////////////////////////////////////////////
        bool project( const unsigned int nNumPoints,
                      const float* pPoints3D,
                      float* pPoints2D,
                      float* pd2d3d,
                      float* pd2dParam,
                      bool bColMajor
                      ) const {
            return project<float>( nNumPoints, pPoints3D, pPoints2D,
                                   pd2d3d, pd2dParam, bColMajor );
        }

        ////////////////////////////////////////////////////////////////////////
        template<class T>
            bool project( const unsigned int nNumPoints,
                          const T* pPoints3D,
                          T* pPoints2D,
                          bool bColMajor ) const {
            if( bColMajor ) {
                return project_col_major<T,false,false>( nNumPoints, pPoints3D, pPoints2D, NULL, NULL );
            }
            else {
                return project_row_major<T,false,false>( nNumPoints, pPoints3D, pPoints2D, NULL, NULL );
            }
        }

        ////////////////////////////////////////////////////////////////////////
        template<class T>
            bool project( const unsigned int nNumPoints,
                          const T* pPoints3D,
                          T* pPoints2D,
                          T* pd2d3d,
                          bool bColMajor = true ) const {
            if( bColMajor ) {
                return project_col_major<T,true,false>( nNumPoints, pPoints3D, pPoints2D, pd2d3d, NULL );
            }
            else {
                 return project_row_major<T,true,false>( nNumPoints, pPoints3D, pPoints2D, pd2d3d, NULL );
            }
        }

        ////////////////////////////////////////////////////////////////////////
        template<class T>
            bool project( const unsigned int nNumPoints,
                          const T* pPoints3D,
                          T* pPoints2D,
                          T* pd2d3d,
                          T* pd2dParam,
                          bool bColMajor = true ) const {
            if( bColMajor ) {
                return project_col_major<T,true,true>( nNumPoints, pPoints3D, pPoints2D, pd2d3d, pd2dParam );
            }
            else {
                 return project_row_major<T,true,true>( nNumPoints, pPoints3D, pPoints2D, pd2d3d, pd2dParam );
            }
        }      

        ////////////////////////////////////////////////////////////////////////
        template<class T, bool COMPUTE_JAC_E, bool COMPUTE_JAC_I>
            bool project_col_major( const unsigned int nNumPoints,
                                    const T* pPoints3D,
                                    T* pPoints2D,
                                    T* pd2d3d,
                                    T* pd2dParam ) const { 
            const int nNumParams = 2*get_num_parameters();
            const int nNumPoseParams = 2*3;
            switch( m_Settings.to_ulong() ) {
            case 0b11110000:
                for( size_t ii=0; ii<nNumPoints; ii++ ) {
                    CPROJECTIONS::project<T,0,0,0,0,COMPUTE_JAC_E,0,0>
                        ( m_dfx, m_dfy, m_dcx, m_dcy, m_dk1, m_dk2, m_dp1, m_dp2, 
                          pPoints3D[3*ii], pPoints3D[3*ii+1], pPoints3D[3*ii+2],
                          &pPoints2D[2*ii], &pPoints2D[2*ii+1],
                          &pd2d3d[nNumPoseParams*ii], &pd2dParam[nNumParams*ii]
                          );
                }
                break;
            case 0b11111000:
                for( size_t ii=0; ii<nNumPoints; ii++ ) {
                    CPROJECTIONS::project<T,1,0,0,0,COMPUTE_JAC_E,COMPUTE_JAC_I,0>
                        ( m_dfx, m_dfy, m_dcx, m_dcy, m_dk1, m_dk2, m_dp1, m_dp2,
                          pPoints3D[3*ii], pPoints3D[3*ii+1], pPoints3D[3*ii+2],
                          &pPoints2D[2*ii], &pPoints2D[2*ii+1],
                          &pd2d3d[nNumPoseParams*ii], &pd2dParam[nNumParams*ii]
                          );
                }
                break;
            case 0b11111100:
                for( size_t ii=0; ii<nNumPoints; ii++ ) {
                    CPROJECTIONS::project<T,1,1,0,0,COMPUTE_JAC_E,COMPUTE_JAC_I,0>
                        ( m_dfx, m_dfy, m_dcx, m_dcy, m_dk1, m_dk2, m_dp1, m_dp2,
                          pPoints3D[3*ii], pPoints3D[3*ii+1], pPoints3D[3*ii+2],
                          &pPoints2D[2*ii], &pPoints2D[2*ii+1],
                          &pd2d3d[nNumPoseParams*ii], &pd2dParam[nNumParams*ii] );
                }
                break;
            case 0b11111110:
                for( size_t ii=0; ii<nNumPoints; ii++ ) {
                    CPROJECTIONS::project<T,1,1,1,0,COMPUTE_JAC_E,COMPUTE_JAC_I,0>
                        ( m_dfx, m_dfy, m_dcx, m_dcy, m_dk1, m_dk2, m_dp1, m_dp2,
                          pPoints3D[3*ii], pPoints3D[3*ii+1], pPoints3D[3*ii+2],
                          &pPoints2D[2*ii], &pPoints2D[2*ii+1], 
                          &pd2d3d[nNumPoseParams*ii], &pd2dParam[nNumParams*ii] );
                }
                break; 
            case 0b11111111:
                for( size_t ii=0; ii<nNumPoints; ii++ ) {
                    CPROJECTIONS::project<T,1,1,1,1,COMPUTE_JAC_E,COMPUTE_JAC_I,0>
                        ( m_dfx, m_dfy, m_dcx, m_dcy, m_dk1, m_dk2, m_dp1, m_dp2,
                          pPoints3D[3*ii], pPoints3D[3*ii+1], pPoints3D[3*ii+2],
                          &pPoints2D[2*ii], &pPoints2D[2*ii+1], 
                          &pd2d3d[nNumPoseParams*ii], &pd2dParam[nNumParams*ii] );
                }
                break;
            default:
                std::cerr << "ERROR: in PinholeRadTanCamera::project unknown settings of parameters." << std::endl;
                return false;
            }
            return true;
        }

        ////////////////////////////////////////////////////////////////////////
        template<class T, bool COMPUTE_JAC_E, bool COMPUTE_JAC_I>
            bool project_row_major( const unsigned int nNumPoints,
                                    const T* pPoints3D,
                                    T* pPoints2D,
                                    T* pd2d3d,
                                    T* pd2dParam ) const {
            const int nNumParams = 2*get_num_parameters();
            const int nNumPoseParams = 6;
            switch( m_Settings.to_ulong() ) {
            case 0b11110000:
                for( size_t ii=0; ii<nNumPoints; ii++ ) {
                    CPROJECTIONS::project<T,0,0,0,0,COMPUTE_JAC_E,0,0>
                        ( m_dfx, m_dfy, m_dcx, m_dcy, m_dk1, m_dk2, m_dp1, m_dp2, 
                          pPoints3D[ii], pPoints3D[ii+nNumPoints], pPoints3D[ii+2*nNumPoints],
                          &pPoints2D[ii], &pPoints2D[ii+nNumPoints], 
                          &pd2d3d[ nNumPoseParams*ii ], &pd2dParam[ nNumParams*ii ]
                          );
                }
                break;
            case 0b11111000:
                for( size_t ii=0; ii<nNumPoints; ii++ ) {
                    CPROJECTIONS::project<T,1,0,0,0,COMPUTE_JAC_E,COMPUTE_JAC_I,0>
                        ( m_dfx, m_dfy, m_dcx, m_dcy, m_dk1, m_dk2, m_dp1, m_dp2,
                          pPoints3D[ii], pPoints3D[ii+nNumPoints], pPoints3D[ii+2*nNumPoints],
                          &pPoints2D[ii], &pPoints2D[ii+nNumPoints], 
                          &pd2d3d[ nNumPoseParams*ii ], &pd2dParam[ nNumParams*ii ]
                          );
                }
                break;
            case 0b11111100:
                for( size_t ii=0; ii<nNumPoints; ii++ ) {
                    CPROJECTIONS::project<T,1,1,0,0,COMPUTE_JAC_E,COMPUTE_JAC_I,0>
                        ( m_dfx, m_dfy, m_dcx, m_dcy, m_dk1, m_dk2, m_dp1, m_dp2,
                          pPoints3D[ii], pPoints3D[ii+nNumPoints], pPoints3D[ii+2*nNumPoints],
                          &pPoints2D[ii], &pPoints2D[ii+nNumPoints], 
                          &pd2d3d[ nNumPoseParams*ii ], &pd2dParam[ nNumParams*ii ]
                          );
                }
                break;
            case 0b11111110:
                for( size_t ii=0; ii<nNumPoints; ii++ ) {
                    CPROJECTIONS::project<T,1,1,1,0,COMPUTE_JAC_E,COMPUTE_JAC_I,0>
                        ( m_dfx, m_dfy, m_dcx, m_dcy, m_dk1, m_dk2, m_dp1, m_dp2,
                          pPoints3D[ii], pPoints3D[ii+nNumPoints], pPoints3D[ii+2*nNumPoints],
                          &pPoints2D[ii], &pPoints2D[ii+nNumPoints], 
                          &pd2d3d[ nNumPoseParams*ii ], &pd2dParam[ nNumParams*ii ] );
                }
                break; 
            case 0b11111111:
                for( size_t ii=0; ii<nNumPoints; ii++ ) {
                    CPROJECTIONS::project<T,1,1,1,1,COMPUTE_JAC_E,COMPUTE_JAC_I,0>
                        ( m_dfx, m_dfy, m_dcx, m_dcy, m_dk1, m_dk2, m_dp1, m_dp2,
                          pPoints3D[ii], pPoints3D[ii+nNumPoints], pPoints3D[ii+2*nNumPoints],
                          &pPoints2D[ii], &pPoints2D[ii+nNumPoints], 
                          &pd2d3d[ nNumPoseParams*ii ], &pd2dParam[ nNumParams*ii ] );
                }
                break;
            default:
                std::cerr << "ERROR: in PinholeRadTanCamera::project unknown settings of parameters." << std::endl;
                return false;
            }
            if( COMPUTE_JAC_E ) {
                transpose( pd2d3d, 2, nNumPoints*nNumPoseParams/2 );
            }
            if( COMPUTE_JAC_I ) {
                transpose( pd2dParam, 2, nNumParams * nNumPoints/2 );
            }
            return true;
        }

#if 0
        ////////////////////////////////////////////////////////////////////////
        PinholeRadTanCamera* new_rectified_camera( const int nImageWidth, 
                                                   const int nImageHeight ) {
#if 0//def CCAMERAMODEL_HAS_OPENCV
            double dAlpha = 0;
            CvMat* intrinsic_matrix      = cvCreateMat(3,3,CV_32FC1);
            CvMat* distortion_coeffs     = cvCreateMat(4,1,CV_32FC1);
            CvMat* new_intrinsic_matrix  = cvCreateMat(3,3,CV_32FC1);

            cvSetZero( intrinsic_matrix );
            CV_MAT_ELEM( *intrinsic_matrix, float, 0, 0 )  = m_dfx;
            CV_MAT_ELEM( *intrinsic_matrix, float, 1, 1 )  = m_dfy;
            CV_MAT_ELEM( *intrinsic_matrix, float, 0, 2 )  = m_dcx;
            CV_MAT_ELEM( *intrinsic_matrix, float, 1, 2 )  = m_dcy;
            CV_MAT_ELEM( *distortion_coeffs, float, 0, 0 ) = m_dk1;
            CV_MAT_ELEM( *distortion_coeffs, float, 1, 0 ) = m_dk2;
            CV_MAT_ELEM( *distortion_coeffs, float, 2, 0 ) = m_dp1;
            CV_MAT_ELEM( *distortion_coeffs, float, 3, 0 ) = m_dp2;

            cvGetOptimalNewCameraMatrix( intrinsic_matrix, 
                                         distortion_coeffs,
                                         cvSize( nImageWidth, nImageHeight ), dAlpha, 
                                         new_intrinsic_matrix );

            PinholeRadTanCamera* pCam = new PinholeRadTanCamera();
            pCam->set<double>( "fx", CV_MAT_ELEM( *new_intrinsic_matrix, float, 0, 0 ) );
            pCam->set<double>( "fy", CV_MAT_ELEM( *new_intrinsic_matrix, float, 1, 1 ) );
            pCam->set<double>( "cx", CV_MAT_ELEM( *new_intrinsic_matrix, float, 0, 2 ) );
            pCam->set<double>( "cy", CV_MAT_ELEM( *new_intrinsic_matrix, float, 1, 2 ) );

            cvReleaseMat( &intrinsic_matrix );
            cvReleaseMat( &distortion_coeffs );
            cvReleaseMat( &new_intrinsic_matrix );
            return pCam;
#else
            std::cerr << "ERROR: not implemented (exits if CCAMERAMODEL_HAS_OPENCV is defined." << std::endl;
            return NULL;
#endif
        } 
#endif

        ////////////////////////////////////////////////////////////////////////
        int get_num_parameters() const {
            return PINHOLE_RADTAN_MAX_NUM_PARAMS;
        }

        ////////////////////////////////////////////////////////////////////////
        int name_to_parameter_index( const std::string& sName ) const {
            if( sName == "fx" ) { return 0; }
            if( sName == "fy" ) { return 1; }
            if( sName == "cx" ) { return 2; }
            if( sName == "cy" ) { return 3; }
            if( sName == "k1" ) { return 4; }
            if( sName == "k2" ) { return 5; }
            if( sName == "p1" ) { return 6; }
            if( sName == "p2" ) { return 7; }
            return -1;
        }

        ////////////////////////////////////////////////////////////////////////
        std::string parameter_index_to_name( const int nIndex ) const {
            if( nIndex == 0 ) { return "fx"; }
            if( nIndex == 1 ) { return "fy"; }
            if( nIndex == 2 ) { return "cx"; }
            if( nIndex == 3 ) { return "cy"; }
            if( nIndex == 4 ) { return "k1"; }
            if( nIndex == 5 ) { return "k2"; }
            if( nIndex == 6 ) { return "p1"; }
            if( nIndex == 7 ) { return "p2"; }
            return "";
        }

        ////////////////////////////////////////////////////////////////////////
        bool initialise_parameters( const int nImageWidth,
                                    const int nImageHeight ) {
            m_nImageWidth  = nImageWidth;
            m_nImageHeight = nImageHeight;
            m_dfx = nImageWidth;
            m_dfy = nImageHeight;
            m_dcx = nImageWidth/2;
            m_dcy = nImageHeight/2;
            m_dk1 = 0; m_dk2 = 0; m_dp1 = 0; m_dp2 = 0;
            m_Settings = 0;
            m_Settings.flip();
            return true;
        }

        ////////////////////////////////////////////////////////////////////////
        void info() const { 
            std::cout << "Camera model combining pinhole with 2 radial and 2 tangential distortion parameters." << std::endl; 
            std::cout << "The following variables can be set/get: fx, fy, cx, cy, k1, k2, p1, p2." << std::endl;
        } 

        ////////////////////////////////////////////////////////////////////////
        std::string get( const std::string& sKey ) const {
            if( sKey == "fx" ) { return ToString<double>( m_dfx ); }
            if( sKey == "fy" ) { return ToString<double>( m_dfy ); }
            if( sKey == "cx" ) { return ToString<double>( m_dcx ); }
            if( sKey == "cy" ) { return ToString<double>( m_dcy ); }
            if( sKey == "k1" ) { return ToString<double>( m_dk1 ); }
            if( sKey == "k2" ) { return ToString<double>( m_dk2 ); }
            if( sKey == "p1" ) { return ToString<double>( m_dp1 ); }
            if( sKey == "p2" ) { return ToString<double>( m_dp2 ); }
            if( sKey == "ImageWidth" ) { return ToString<int>( m_nImageWidth ); }
            if( sKey == "ImageHeight" ) { return ToString<int>( m_nImageHeight ); }
            return "";
        }

        ////////////////////////////////////////////////////////////////////////
        bool set( const std::string& sKey, const std::string& sParameter ) {
            if( sKey == "fx"      ) { m_dfx = FromString<double>( sParameter ); }
            else if( sKey == "fy" ) { m_dfy = FromString<double>( sParameter ); }
            else if( sKey == "cx" ) { m_dcx = FromString<double>( sParameter ); }
            else if( sKey == "cy" ) { m_dcy = FromString<double>( sParameter ); }
            else if( sKey == "k1" ) {
                m_Settings[K1_INDEX] = 1;
                m_dk1 = FromString<double>( sParameter );
            }
            else if( sKey == "k2" ) {
                m_Settings[K2_INDEX] = 1;
                m_dk2 = FromString<double>( sParameter );
            }
            else if( sKey == "p1" ) {
                m_Settings[P1_INDEX] = 1;
                m_dp1 = FromString<double>( sParameter );
            }
            else if( sKey == "p2" ) {
                m_Settings[P2_INDEX] = 1;
                m_dp2 = FromString<double>( sParameter );
            } 
            else if( sKey == "ImageWidth" ) { m_nImageWidth = FromString<int>( sParameter ); }
            else if( sKey == "ImageHeight" ) { m_nImageHeight = FromString<int>( sParameter ); }
            else {
                return false;
            }
            return true;
        }

    private:
        std::bitset<PINHOLE_RADTAN_MAX_NUM_PARAMS> m_Settings;
        double m_dfx, m_dfy, m_dcx, m_dcy;
        double m_dk1, m_dk2, m_dp1, m_dp2;
        int m_nImageWidth, m_nImageHeight;
    };
}

#endif
