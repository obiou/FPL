#include <CCameraModel/CameraModel.h>
#include <CCameraModel/CameraStringParsing.h>

#if CCAMERAMODEL_HAS_EIGEN3
#  include <eigen3/Eigen/Cholesky>
#  include <eigen3/Eigen/Dense>
#endif

#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>

namespace CCameraModel {
    ////////////////////////////////////////////////////////////////////////////
    class NoMask {
    public: 
        NoMask() {}
        NoMask( int, int ) {}
        ~NoMask() {}
        void fwd() {}
        void set( bool ) {}
        void end() {}
        void set_to_nan( float*, int, int ) {}
    };

    ////////////////////////////////////////////////////////////////////////////
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
            *pWarpedPatch = (char) ( fV00 +
                                     (fXc-nFloorXc)*(fV10-fV00)+
                                     (fYc-nFloorYc)*(fV01-fV00)-
                                     (fXc-nFloorXc)*(fYc-nFloorYc)*(fV01-fV00+fV10-fV11) );
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

    ////////////////////////////////////////////////////////////////////////////
    template <class T> std::string ToString( const T& param ) {
        std::ostringstream os;
        os << std::setprecision( 11 ) << std::setiosflags( std::ios::fixed ) << param;
        return os.str();
    }

    ////////////////////////////////////////////////////////////////////////////
    template <class T> T FromString( const std::string& s ) {
        std::stringstream ss;
        ss << s; T t; ss >> t;
        return t;
    }

    ////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////
    bool CameraModel::save( const std::string& sFileName ) {
        if( m_pCameraModel == NULL ) {
            return false;
        }
        std::ofstream outFile( sFileName.c_str() );
        outFile << *m_pCameraModel;
        outFile.close();
        return true;
    }
    
    ////////////////////////////////////////////////////////////////////////////
#define MAKE_PARSING_INDEPENDENT_OF_ORDER 0

    ////////////////////////////////////////////////////////////////////////////
    bool CameraModel::load( const std::string& sFileName ) {
        std::ifstream inFile( sFileName.c_str() );
        // Look for camera type
        std::string sCameraType;
        const std::string sCameraTypePrefix = "cameraModel.CameraType";
#if MAKE_PARSING_INDEPENDENT_OF_ORDER
        // Copy stream to avoid loosing data (or making parsing dependent on order)
        // when calling FindCameraType, better way? (Could be very slow with LUT)
        std::stringstream sStream;
        sStream << inFile.rdbuf();
        bool bFoundCameraType = FindCameraType( sStream, sCameraTypePrefix, sCameraType );
        
#else
        bool bFoundCameraType = FindCameraType( inFile, sCameraTypePrefix, sCameraType );
#endif
        if( bFoundCameraType ) {
            //std::cout << "Found camera: " << sCameraType << std::endl;
        }
        else {
            std::cerr << "ERROR in CameraModelInterface::FromStream, ";
            std::cerr << "did not find camera type when parsing." << std::endl;
            return false;
        }
        // Make new m_pCameraModel if necessary
        if( m_pCameraModel != NULL &&
            m_pCameraModel->get_camera_type() != sCameraType ) {
            delete m_pCameraModel;
            m_pCameraModel = NULL;
        }
        if( m_pCameraModel == NULL ) {
            set_camera_type( sCameraType );
            m_pCameraModel = CameraModelInterfaceFactory::Create( get_camera_type() );
            m_pCameraModel->set_camera_type( sCameraType );
        }
#if MAKE_PARSING_INDEPENDENT_OF_ORDER
        sStream >> *m_pCameraModel;
#else
        inFile >> *m_pCameraModel;
#endif
        inFile.close();
        return true;
    }

#if CCAMERAMODEL_HAS_EIGEN3
    ////////////////////////////////////////////////////////////////////////
    /// Transform points using a rotation and translation and 
    /// project to image
    template<class Derived1,class Derived2, class Derived3, 
             class Derived4, class Derived5>
        bool CameraModel::project_trans
        ( const Eigen::MatrixBase<Derived1>& mR,
          const Eigen::MatrixBase<Derived2>& mt,
          const Eigen::MatrixBase<Derived3>& mP3D,
          Eigen::MatrixBase<Derived4>& mP2D,  
          Eigen::MatrixBase<Derived5>& mdP2DE
          ) const {
        typedef typename Eigen::internal::traits<Derived4>::Scalar Derived4Type;
        typedef typename Eigen::internal::traits<Derived5>::Scalar Derived5Type;
        typedef Eigen::Matrix<Derived4Type, 1, 3> Derived4Vec;
        assert( mP3D.rows() == 3 );
        assert( mP2D.rows() == 2 );
        assert( mP3D.cols() == mP2D.cols() );
        assert( mR.rows() == 3 );
        assert( mR.cols() == 3 );
        assert( mt.rows() == 3 );
        assert( mt.cols() == 1 );
        Derived3 mP3DT = mR * mP3D;
        mP3DT.colwise() += mt;
        Eigen::Matrix<Derived5Type,2,Eigen::Dynamic,Derived5::Base::Options> mdP2DP3D( 2, 3*mP3D.cols() );
        bool bSuccess = project( mP3DT, mP2D, mdP2DP3D );
        if( !bSuccess ) { return bSuccess; }
        Derived4 mdR0 = mR*CEIGEN::skew_rot<Derived4>(1,0,0);
        Derived4 mdR1 = mR*CEIGEN::skew_rot<Derived4>(0,1,0);
        Derived4 mdR2 = mR*CEIGEN::skew_rot<Derived4>(0,0,1);
            
        for( int ii=0; ii<mP3D.cols(); ii++ ) {
            // Chain rule: right part of the extrinsic matrix
            // computation related to the rotation and translation
            Derived4 mdRtX( 3, 6 );
            // Rotation part
            mdRtX.block(0,0,3,1) = mdR0*mP3D.col( ii );
            mdRtX.block(0,1,3,1) = mdR1*mP3D.col( ii );
            mdRtX.block(0,2,3,1) = mdR2*mP3D.col( ii );
            // Translation part
            mdRtX.block(0,3,3,3) = Derived4::Identity(3,3);
            // Combine extrinsic Jacobian with position Jacobian (right hand side)
            mdP2DE.block(0,6*ii,2,6) = mdP2DP3D.block(0,3*ii,2,3) * mdRtX;
        }
        return true;
    }

    ////////////////////////////////////////////////////////////////////////
    /// Transform points using a rotation and translation and 
    /// project to image
    template<class Derived1,class Derived2, class Derived3, 
             class Derived4, class Derived5, class Derived6>
        bool CameraModel::project_trans
        ( const Eigen::MatrixBase<Derived1>& mR,
          const Eigen::MatrixBase<Derived2>& mt,
          const Eigen::MatrixBase<Derived3>& mP3D,
          Eigen::MatrixBase<Derived4>& mP2D,  
          Eigen::MatrixBase<Derived5>& mdP2DE,
          Eigen::MatrixBase<Derived6>& mdP2DI 
          ) const {
        typedef typename Eigen::internal::traits<Derived4>::Scalar Derived4Type;
        typedef typename Eigen::internal::traits<Derived5>::Scalar Derived5Type;
        typedef Eigen::Matrix<Derived4Type, 1, 3> Derived4Vec;
        assert( mP3D.rows() == 3 );
        assert( mP2D.rows() == 2 );
        assert( mP3D.cols() == mP2D.cols() );
        assert( mR.rows() == 3 );
        assert( mR.cols() == 3 );
        assert( mt.rows() == 3 );
        assert( mt.cols() == 1 );
        Derived3 mP3DT = mR * mP3D;
        mP3DT.colwise() += mt;
        Eigen::Matrix<Derived5Type,2,Eigen::Dynamic,Derived5::Base::Options> mdP2DP3D( 2, 3*mP3D.cols() );
        bool bSuccess = project( mP3DT, mP2D, mdP2DP3D, mdP2DI );
        if( !bSuccess ) { return bSuccess; }
        Derived4 mdR0 = mR*CEIGEN::skew_rot<Derived4>(1,0,0);
        Derived4 mdR1 = mR*CEIGEN::skew_rot<Derived4>(0,1,0);
        Derived4 mdR2 = mR*CEIGEN::skew_rot<Derived4>(0,0,1);
            
        for( int ii=0; ii<mP3D.cols(); ii++ ) {
            // Chain rule: right part of the extrinsic matrix
            // computation related to the rotation and translation
            Derived4 mdRtX( 3, 6 );
            // Rotation part
            mdRtX.block(0,0,3,1) = mdR0*mP3D.col( ii );
            mdRtX.block(0,1,3,1) = mdR1*mP3D.col( ii );
            mdRtX.block(0,2,3,1) = mdR2*mP3D.col( ii );
            // Translation part
            mdRtX.block(0,3,3,3) = Derived4::Identity(3,3);
            // Combine extrinsic Jacobian with position Jacobian (right hand side)
            mdP2DE.block(0,6*ii,2,6) = mdP2DP3D.block(0,3*ii,2,3) * mdRtX;
        }
        return true;
    }

#if 1
    ////////////////////////////////////////////////////////////////////////////
    template<class Derived1,class Derived2, class Derived3>
            bool CameraModel::_project_extrinsic( const Eigen::MatrixBase<Derived1>& mP3D,
                                                  Eigen::MatrixBase<Derived2>& mP2D,
                                                  Eigen::MatrixBase<Derived3>& mdP2DE
                                                  ) const {
        if( !project( mP3D, mP2D ) ) {
            return false;
        }
        const unsigned int nNum3DDims = 3;
        const unsigned int nNumPoints = mP3D.cols();
        // Compute finite-diff Jacobians
        Derived3 mdP2Dfd( 2, nNumPoints );
        Derived3 mP2Dfd( 2, nNumPoints );
        double dEps = 1e-4;
        Derived2 mP2DP = mP2D;
        Derived2 mP2DM = mP2D;
        { 
            // Compute extrinsic Jacobian
            for( size_t kk=0; kk<nNum3DDims; kk++ ) {
                Derived1 mP3DP = mP3D;
                Derived1 mP3DM = mP3D;
                mP3DP.row( kk ).array() += dEps;
                mP3DM.row( kk ).array() -= dEps;
                project( mP3DP, mP2DP );
                project( mP3DM, mP2DM );
                Derived2 mdP2Dfd = ( mP2DP - mP2DM ) / (2*dEps);
                for( size_t ll=0; ll<nNumPoints; ll++ ) {
                    mdP2DE.col( nNum3DDims*ll + kk ) = mdP2Dfd.col( ll );
                }
            }
        }
        return true;
    }

    ////////////////////////////////////////////////////////////////////////////
    template<class Derived1,class Derived2, class Derived3>
            bool CameraModel::_project_intrinsic( const Eigen::MatrixBase<Derived1>& mP3D,
                                                  Eigen::MatrixBase<Derived2>& mP2D,
                                                  Eigen::MatrixBase<Derived3>& mdP2DI
                                                  ) const { 
        if( !project( mP3D, mP2D ) ) {
            return false;
        }
        const unsigned int nNumPoints = mP3D.cols();
        const unsigned int nNumCamParams = get_num_parameters();
        // Compute finite-diff Jacobians
        Derived3 mdP2Dfd( 2, nNumPoints );
        Derived3 mP2Dfd( 2, nNumPoints );
        double dEps = 1e-4;
        Derived2 mP2DP = mP2D;
        Derived2 mP2DM = mP2D;
        { 
            // Compute intrinsic Jacobian
            // Horrible const_cast ... difficult to avoid
            for( size_t kk=0; kk<nNumCamParams; kk++ ) {
                std::string sVarName = parameter_index_to_name( kk );
                double dVal = get<double>( sVarName );
                const_cast<CameraModel*>( this )->set<double>( sVarName, dVal + dEps );
                project( mP3D, mP2DP );
                const_cast<CameraModel*>( this )->set<double>( sVarName, dVal - dEps );
                project( mP3D, mP2DM );
                mdP2Dfd = ( mP2DP - mP2DM ) / (2*dEps);
                const_cast<CameraModel*>( this )->set<double>( sVarName, dVal );
                for( size_t ll=0; ll<nNumPoints; ll++ ) {
                    mdP2DI.col( nNumCamParams*ll + kk ) = mdP2Dfd.col( ll );
                }
            }
        }
        return true;
    }

    ////////////////////////////////////////////////////////////////////////////
    template<class Derived1,class Derived2, class Derived3, class Derived4>
            bool CameraModel::_project( const Eigen::MatrixBase<Derived1>& mP3D,
                                        Eigen::MatrixBase<Derived2>& mP2D,
                                        Eigen::MatrixBase<Derived3>& mdP2DE,
                                        Eigen::MatrixBase<Derived4>& mdP2DI
                                        ) const {
        return _project_extrinsic( mP3D, mP2D, mdP2DE ) &&
            _project_intrinsic( mP3D, mP2D, mdP2DI );
    }
#endif

    ////////////////////////////////////////////////////////////////////////////
    template<class Derived1, class Derived2>
        bool CameraModel::_lift( const Eigen::MatrixBase<Derived1>& mP2D,
                                 Eigen::MatrixBase<Derived2>& mP3D ) const {
        typedef typename Eigen::internal::traits<Derived2>::Scalar ScalarType;
        const unsigned int nNumPoints = mP2D.cols();
#if 0
        typedef Eigen::Matrix<ScalarType,1,Eigen::Dynamic> RowVect;
        const int nWidth = 1000;
        const int nHeight = 1000;
        const int nWidthSize = nWidth/10;
        const int nHeightSize = nHeight/10;
        Derived1 mP2DUndist( 3, nWidthSize*nHeightSize );
        Derived1 m1 = RowVect::LinSpaced( nWidthSize, 0, nWidth-1 ).replicate( nHeightSize, 1 );
        mP2DUndist.row( 0 ) = Eigen::Map<Derived1>( m1.data(), 1, nWidthSize*nHeightSize );
        mP2DUndist.row( 1 ) = RowVect::LinSpaced( nHeightSize, 0, nHeight-1 ).replicate( 1, nWidthSize );
        mP2DUndist.row( 2 ) = RowVect::Ones( 1, nWidthSize*nHeightSize );
        Derived1 mP2DDist( 2, nWidthSize*nHeightSize );
        project( mP2DUndist, mP2DDist );

        // Initialise with closest values
        for( int ii=0; ii<mP2D.cols(); ii++ ) {
            //Derived1::Index nMinIndex;
            int nMinIndex;
            Derived1 mDiff = mP2DDist;
            mDiff.colwise() -= mP2D.col( ii );
            mDiff.colwise().norm().minCoeff( &nMinIndex );
            mP3D.col( ii ) = mP2DUndist.col( nMinIndex );
        }
#else
        // Start from distortionless points if possible
        if( get( "fx" ) != "" && get( "fy" ) != "" && 
            get( "cx" ) != "" && get( "cy" ) != "" ) {
            double fx = get<double>( "fx" ); double fy = get<double>( "fy" );
            double cx = get<double>( "cx" ); double cy = get<double>( "cy" );
            mP3D.row( 0 ) = ( mP2D.row( 0 ) - cx*Derived2::Ones( 1, nNumPoints ) ) / fx;
            mP3D.row( 1 ) = ( mP2D.row( 1 ) - cy*Derived2::Ones( 1, nNumPoints ) ) / fy;
            mP3D.row( 2 ) = Derived2::Ones( 1, nNumPoints );            
        }
        else {
            mP3D.row( 0 ) = mP2D.row( 0 );
            mP3D.row( 1 ) = mP2D.row( 1 );
            mP3D.row( 2 ) = Derived2::Ones( 1, nNumPoints );
        }
#endif
        for( size_t nIndex=0; nIndex<nNumPoints; nIndex++ ) {
            mP3D.col( nIndex ) /= mP3D.col( nIndex ).norm();
        }

        Derived1 mP2DEst( 2, nNumPoints );
        Derived2 mdP2DE( 2, 3*nNumPoints );
        Derived2 mdP2DI( 2, get_num_parameters()*nNumPoints );

        double dMaxErr = 0.001;
        double dLastMaxErr = 10;
        const unsigned int nMaxIter = 30;
        const unsigned int nMaxOuterIter = 5;
        for( size_t nOuterIter=0; nOuterIter<nMaxOuterIter; nOuterIter++ ) {
            for( size_t nIter=0; nIter<nMaxIter; nIter++ ) {
                project( mP3D, mP2DEst, mdP2DE, mdP2DI );
                for( size_t nIndex=0; nIndex<nNumPoints; nIndex++ ) {
                    Eigen::Matrix<ScalarType,2,3> mJ = mdP2DE.block( 0, 3*nIndex, 2, 3 );
                    Eigen::Matrix<ScalarType,3,3> mJtJ = mJ.transpose()*mJ + 0.1*Eigen::Matrix<ScalarType,3,3>::Identity();
                    Eigen::Matrix<ScalarType,3,1> mJte = mJ.transpose() * ( mP2DEst.col( nIndex ) - mP2D.col( nIndex ) );
                    mP3D.col( nIndex ) -= mJtJ.ldlt().solve( mJte );
                
                    double dErr = ( mP2DEst.col( nIndex ) - mP2D.col( nIndex ) ).norm();
                    if( nIndex > 0 ) {
                        if( std::isnan( dErr ) ) {
                            mP3D.col( nIndex ) =  mP3D.col( nIndex-1 );
                        }
                    }
                    mP3D.col( nIndex ) /= mP3D.col( nIndex ).norm();
                }
                dLastMaxErr = ( mP2DEst - mP2D ).colwise().norm().maxCoeff();
                if( dLastMaxErr < dMaxErr ) {
                    break;
                }
            }

            if( dLastMaxErr >= dMaxErr ) {
                Derived1 mErrors = ( mP2DEst - mP2D ).colwise().norm();
                for( int ii=0; ii<mErrors.cols(); ii++ ) {
                    if( mErrors(0,ii) > dMaxErr ) {
                        // Find closest value
                        double dMinDiff = 1e10;
                        int nBestIndex = -1;
                        for( int jj=0; jj<mErrors.cols(); jj++ ) {
                            if( jj != ii && mErrors(0,jj) < dMaxErr ) {
                                double dDiff = ( mP2D.col( ii ) - mP2D.col( jj ) ).norm();
                                if( dDiff <  dMinDiff ) { 
                                    dMinDiff = dDiff;
                                    nBestIndex = jj;
                                }
                            }
                        }
                        if( nBestIndex == -1 ) {
                            // All is lost, could not find an index 
                            // that passes the error test
                            return false;
                        }
                        mP3D.col( ii ) = mP3D.col( nBestIndex ) ;
                    }
                }
            }
        }

        return dLastMaxErr < dMaxErr;
    }

#endif

#if CCAMERAMODEL_HAS_EIGEN3
    ////////////////////////////////////////////////////////////////////////////////
    CameraModel* CameraModel::new_rectified_camera
    ( const int nImageWidth, 
      const int nImageHeight,
      const std::vector<double>* pvParams
      ) {
#if 0
        CameraModelInterface<>* pCam = NULL;
        if( m_pCameraModel != NULL ) { 
            pCam = m_pCameraModel->new_rectified_camera( nImageWidth, nImageHeight,
                                                         pvParams );
        }
#endif
        //if( pCam == NULL ) {
        return _new_rectified_camera( nImageWidth, nImageHeight, pvParams );
        //}        
        //return pCam;
    }

#include <CCameraModel/PinholeCamera.h>
    ////////////////////////////////////////////////////////////////////////////////
    bool ProjectInImage( const int nImageWidth, 
                         const int nImageHeight,
                         const double dFx, const double dFy, 
                         const double dCx, const double dCy,
                         CameraModel* pThis ) {
        CameraModel* pCam = new CameraModel( "Pinhole" );
        pCam->set<double>( "fx", dFx );
        pCam->set<double>( "fy", dFy );
        pCam->set<double>( "cx", dCx );
        pCam->set<double>( "cy", dCy );

        // Create border pixel coordinates
        Eigen::MatrixXd mBorder = CEIGEN::make_border( nImageWidth, nImageHeight );
        Eigen::MatrixXd mLift( 3, 2*nImageWidth + 2*nImageHeight );
        Eigen::MatrixXd mReProj( 2, 2*nImageWidth + 2*nImageHeight );
        pCam->lift( mBorder, mLift );
        
        using namespace Eigen;

        pThis->project( mLift, mReProj );
        bool bPos = ( mReProj.array() >= 0 ).all();
        bool bXIn = ( mReProj.row(0).array() < nImageWidth ).all();
        bool bYIn = ( mReProj.row(1).array() < nImageHeight ).all();

        delete pCam;
        return bPos && bXIn && bYIn;
    }

    ////////////////////////////////////////////////////////////////////////////////
    CameraModel* CameraModel::_new_rectified_camera
    ( const int nImageWidth, 
      const int nImageHeight,
      const std::vector<double>* pvParams
      ) {
        double dFx, dFy, dCx, dCy;
        if( pvParams != NULL ) {
            if( pvParams->size() < 4 ) {
                std::cerr << "ERROR: in _new_rectified_camera, expecting pvParams to be of size 4." << std::endl;
                return NULL;
            }
            dFx = pvParams->at(0);
            dFy = pvParams->at(0);
            dCx = pvParams->at(0);
            dCy = pvParams->at(0);
        }
        else {
            if( get_num_parameters() < 4 ) {
                std::cerr << "ERROR: in _new_rectified_camera, expecting get_num_parameters to return at least 4." << std::endl;
                return NULL;
            }
            dFx = get<double>( parameter_index_to_name( 0 ) );
            dFy = get<double>( parameter_index_to_name( 1 ) );
            dCx = get<double>( parameter_index_to_name( 2 ) );
            dCy = get<double>( parameter_index_to_name( 3 ) );
        }

        double dMaxScale = 1.2;
        double dMinScale = 0.8;

        while( !ProjectInImage( nImageWidth, nImageHeight, 
                                dMaxScale*dFx,
                                dMaxScale*dFy,
                                dCx, dCy, this ) ) {
            dMaxScale *= 2;
        }

        while( ProjectInImage( nImageWidth, nImageHeight,
                               dMinScale*dFx,
                               dMinScale*dFy,
                               dCx, dCy, this ) ) {
            dMinScale /= 2;
        }
                               
        // Proceed with bisection
        int nMaxNumIters = 15;
        double dMid = 0;
        for( int ii=0; ii<nMaxNumIters; ii++ ) {
            dMid = ( dMinScale + dMaxScale ) / 2;
            if( ProjectInImage( nImageWidth, nImageHeight,
                                dMid*dFx,
                                dMid*dFy,
                                dCx, dCy, this ) ) {
                dMaxScale = dMid;
            }
            else {
                dMinScale = dMid;
            }
            //std::cout << "dMid: " << dMid << std::endl;
        }   
        //std::cout << "dMaxScale: " << dMid << std::endl;
        CameraModel* pCam = new CameraModel( "Pinhole" );
        pCam->set<double>( "fx", dMaxScale*dFx );
        pCam->set<double>( "fy", dMaxScale*dFy );
        pCam->set<double>( "cx", dCx );
        pCam->set<double>( "cy", dCy );
        return pCam;
    } 

    ////////////////////////////////////////////////////////////////////////////
    void CameraModel::undistort_image( const int nImageWidth,
                                       const int nImageHeight,
                                       const int nImageWidthStep,
                                       const unsigned char* pImageIn,
                                       unsigned char* pImageOut,
                                       bool bRecompute ) {
        if( bRecompute || m_mLUT.rows() != 2 || m_mLUT.cols() != nImageWidth*nImageHeight ) {
            std::cout << "Computing LUT... ";
            CameraModel* pCamUndistort = new_rectified_camera( nImageWidth, nImageHeight );
            if( pCamUndistort == NULL ) {
                std::cerr << "ERROR calling new_rectified_camera." << std::endl;
                return;
            }
            // Make image grid
            Eigen::MatrixXf mGrid = CEIGEN::make_grid_f( nImageWidth, nImageHeight, false, false );
            // Lift and reproject
            Eigen::MatrixXf mLift( 3, mGrid.cols() );
            m_mLUT = Eigen::MatrixXf( 2, mGrid.cols() );
            pCamUndistort->lift( mGrid, mLift );
            project( mLift, m_mLUT );          
            std::cout << "done." << std::endl;
            delete pCamUndistort;
        }
        NoMask noMask;
        for( int nRow=0; nRow<nImageHeight; nRow++ ) {
            for( int nCol=0; nCol< nImageWidth; nCol++ ) {
#if 0
                float nNewCol = m_mLUT( 0, nCol + nRow*nImageWidth );
                float nNewRow = m_mLUT( 1, nCol + nRow*nImageWidth );
                pImageOut[ nCol + nRow*nImageWidth ] = 
                    pImageIn[ nNewCol + nNewRow*nImageWidth ];
#endif
                BilinearInterpolation( nImageWidth, nImageHeight, 
                                       nImageWidthStep, pImageIn,
                                       m_mLUT( 0, nCol + nRow*nImageWidth ),
                                       m_mLUT( 1, nCol + nRow*nImageWidth ),
                                       pImageOut, noMask );
                pImageOut++;
            }
            pImageOut += nImageWidthStep - nImageWidth;
        }       
    }

#  if CCAMERAMODEL_HAS_OPENCV
        ////////////////////////////////////////////////////////////////////////
        void CameraModel::undistort_image( const IplImage* pImageIn,
                                           IplImage* pImageOut,
                                           bool bRecompute ) {
            assert( pImageIn->width == pImageOut->width );
            assert( pImageIn->height == pImageOut->height );
            assert( pImageIn->widthStep == pImageOut->widthStep );
            undistort_image( pImageIn->width, pImageIn->height,
                             pImageIn->widthStep,
                             (const unsigned char*)pImageIn->imageData,
                             (unsigned char*)pImageOut->imageData, bRecompute );
        }
#  endif
#endif

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
    // Instantiations
#define INSTANTIATE_TO_STRING( T )              \
    template std::string ToString( const T& );
    
#define INSTANTIATE_FROM_STRING( T )               \
    template T FromString( const std::string& );

#define INSTANTIATE_PROJECT_TRANS( T1, T2, T3, T4, T5, T6 ) \
    template                                                \
    bool CameraModel::project_trans                         \
    ( const Eigen::MatrixBase<T1>&,                         \
      const Eigen::MatrixBase<T2>&,                         \
      const Eigen::MatrixBase<T3>&,                         \
      Eigen::MatrixBase<T4>&,                               \
      Eigen::MatrixBase<T5>&,                               \
      Eigen::MatrixBase<T6>& ) const;                       \
    template                                                \
    bool CameraModel::project_trans                         \
    ( const Eigen::MatrixBase<T1>&,                         \
      const Eigen::MatrixBase<T2>&,                         \
      const Eigen::MatrixBase<T3>&,                         \
      Eigen::MatrixBase<T4>&,                               \
      Eigen::MatrixBase<T5>&                                \
      ) const;

#define INSTANTIATE__PROJECT( T1, T2, T3, T4 )                  \
    template                                                    \
    bool CameraModel::_project( const Eigen::MatrixBase<T1>& ,  \
                                Eigen::MatrixBase<T2>& ,        \
                                Eigen::MatrixBase<T3>& ,        \
                                Eigen::MatrixBase<T4>& ) const; \
   
#define INSTANTIATE__LIFT( T1, T2 )                         \
    template                                                \
    bool CameraModel::_lift( const Eigen::MatrixBase<T1>&,  \
                             Eigen::MatrixBase<T2>& ) const;
    
    INSTANTIATE_TO_STRING( int );
    INSTANTIATE_TO_STRING( float );
    INSTANTIATE_TO_STRING( double );
    INSTANTIATE_FROM_STRING( int );
    INSTANTIATE_FROM_STRING( float );
    INSTANTIATE_FROM_STRING( double );

    typedef Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic,Eigen::RowMajor> MatrixXdRowMajor;
    typedef Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic,Eigen::ColMajor> MatrixXdColMajor;
    typedef Eigen::Matrix<double,Eigen::Dynamic,1,Eigen::RowMajor> VectorXdRowMajor;
    typedef Eigen::Matrix<double,Eigen::Dynamic,1,Eigen::ColMajor> VectorXdColMajor;
    typedef Eigen::Matrix<float,Eigen::Dynamic,Eigen::Dynamic,Eigen::RowMajor> MatrixXfRowMajor;
    typedef Eigen::Matrix<float,Eigen::Dynamic,Eigen::Dynamic,Eigen::ColMajor> MatrixXfColMajor;
    typedef Eigen::Matrix<float,Eigen::Dynamic,1,Eigen::RowMajor> VectorXfRowMajor;
    typedef Eigen::Matrix<float,Eigen::Dynamic,1,Eigen::ColMajor> VectorXfColMajor;

    typedef Eigen::Matrix<double,3,3,Eigen::RowMajor> Matrix3dRowMajor;
    typedef Eigen::Matrix<double,3,3,Eigen::ColMajor> Matrix3dColMajor;
    typedef Eigen::Matrix<double,3,1,Eigen::RowMajor> Vector3dRowMajor;
    typedef Eigen::Matrix<double,3,1,Eigen::ColMajor> Vector3dColMajor;
    typedef Eigen::Matrix<float,3,3,Eigen::RowMajor> Matrix3fRowMajor;
    typedef Eigen::Matrix<float,3,3,Eigen::ColMajor> Matrix3fColMajor;
    typedef Eigen::Matrix<float,3,1,Eigen::RowMajor> Vector3fRowMajor;
    typedef Eigen::Matrix<float,3,1,Eigen::ColMajor> Vector3fColMajor;
   
    INSTANTIATE_PROJECT_TRANS( MatrixXdRowMajor, VectorXdRowMajor, MatrixXdRowMajor,
                               MatrixXdRowMajor, MatrixXdRowMajor, MatrixXdRowMajor );
    INSTANTIATE_PROJECT_TRANS( MatrixXdColMajor, VectorXdColMajor, MatrixXdColMajor,
                               MatrixXdColMajor, MatrixXdColMajor, MatrixXdColMajor );
    INSTANTIATE_PROJECT_TRANS( Matrix3dRowMajor, Vector3dRowMajor, MatrixXdRowMajor,
                               MatrixXdRowMajor, MatrixXdRowMajor, MatrixXdRowMajor );
    INSTANTIATE_PROJECT_TRANS( Matrix3dColMajor, Vector3dColMajor, MatrixXdColMajor,
                               MatrixXdColMajor, MatrixXdColMajor, MatrixXdColMajor );

    INSTANTIATE_PROJECT_TRANS( MatrixXfRowMajor, VectorXfRowMajor, MatrixXfRowMajor,
                               MatrixXfRowMajor, MatrixXfRowMajor, MatrixXfRowMajor );
    INSTANTIATE_PROJECT_TRANS( MatrixXfColMajor, VectorXfColMajor, MatrixXfColMajor,
                               MatrixXfColMajor, MatrixXfColMajor, MatrixXfColMajor );
    INSTANTIATE_PROJECT_TRANS( Matrix3fRowMajor, Vector3fRowMajor, MatrixXfRowMajor,
                               MatrixXfRowMajor, MatrixXfRowMajor, MatrixXfRowMajor );
    INSTANTIATE_PROJECT_TRANS( Matrix3fColMajor, Vector3fColMajor, MatrixXfColMajor,
                               MatrixXfColMajor, MatrixXfColMajor, MatrixXfColMajor );

    INSTANTIATE__PROJECT( MatrixXdRowMajor, MatrixXdRowMajor, MatrixXdRowMajor, MatrixXdRowMajor );
    INSTANTIATE__PROJECT( MatrixXdColMajor, MatrixXdColMajor, MatrixXdColMajor, MatrixXdColMajor );

    INSTANTIATE__PROJECT( MatrixXfRowMajor, MatrixXfRowMajor, MatrixXfRowMajor, MatrixXfRowMajor );
    INSTANTIATE__PROJECT( MatrixXfColMajor, MatrixXfColMajor, MatrixXfColMajor, MatrixXfColMajor );
    
    INSTANTIATE__LIFT( Eigen::MatrixXd, Eigen::MatrixXd );
    INSTANTIATE__LIFT( Eigen::MatrixXf, Eigen::MatrixXf );
}

