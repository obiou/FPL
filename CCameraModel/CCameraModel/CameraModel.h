#ifndef CCAMERA_MODEL_H
#define CCAMERA_MODEL_H

#include <CCameraModelIncludes.h>
#include <CCameraModel/CameraModelInterface.h>

#if CCAMERAMODEL_HAS_EIGEN3
//#  define EIGEN_DEFAULT_TO_ROW_MAJOR 1
#  include <eigen3/Eigen/Core>
#endif

#if CCAMERAMODEL_HAS_OPENCV
#  include <cv.h>
#  include <highgui.h>
#endif

#include <ceigen.h>

#include <string>

////////////////////////////////////////////////////////////////////////////////
namespace CCameraModel {
    ////////////////////////////////////////////////////////////////////////////
    /// Conversion function: converts to a string
    template <class T> std::string ToString( const T& param );
    
    ////////////////////////////////////////////////////////////////////////////
    /// Conversion function: converts from a string
    template <class T> T FromString( const std::string& s );
}

////////////////////////////////////////////////////////////////////////////////
namespace CCameraModel {
    ////////////////////////////////////////////////////////////////////////////
    /// Main class used by the final user
    class CameraModel : public CameraModelInterface<> {
    public:
        ////////////////////////////////////////////////////////////////////////
    CameraModel() :
        m_pCameraModel( NULL ) {}

        ////////////////////////////////////////////////////////////////////////
    CameraModel( const std::string& sCameraType ) : 
        m_pCameraModel( CameraModelInterfaceFactory::Create( sCameraType ) )
            { set_camera_type( sCameraType ); 
                if( m_pCameraModel != NULL ) { 
                    m_pCameraModel->set_camera_type( sCameraType );
                }
            }

        ////////////////////////////////////////////////////////////////////////
        ~CameraModel() {
            if( m_pCameraModel != NULL ) {
                delete m_pCameraModel;
                m_pCameraModel = NULL;
            }
        }

        ////////////////////////////////////////////////////////////////////////
        /// Project points to image
        /// N: number of points
        /// M: number of model parameters
        bool project( const unsigned int nNumPoints, ///<Input: N, the number of points
                      const double* pPoints3D, ///<Input: 3D points
                      double* pPoints2D, ///<Output: 3D points projected into the image 2xN
                      bool bColMajor = true
                      ) const {
            if( m_pCameraModel != NULL ) {
                return m_pCameraModel->project( nNumPoints, pPoints3D, pPoints2D, bColMajor );
            }
            return false;
        }

        ////////////////////////////////////////////////////////////////////////
        /// Project points to image
        /// N: number of points
        /// M: number of model parameters
        bool project( const unsigned int nNumPoints, ///<Input: N, the number of points
                      const float* pPoints3D, ///<Input: 3D points
                      float* pPoints2D, ///<Output: 3D points projected into the image 2xN
                      bool bColMajor = true
                      ) const {
            if( m_pCameraModel != NULL ) {
                return m_pCameraModel->project( nNumPoints, pPoints3D, pPoints2D, bColMajor );
            }
            return false;
        }

#if CCAMERAMODEL_HAS_EIGEN3
        ////////////////////////////////////////////////////////////////////////
        /// Project points to image
        template<class Derived1,class Derived2>
            bool project( const Eigen::MatrixBase<Derived1>& mP3D,
                          Eigen::MatrixBase<Derived2>& mP2D ) const {
            assert( mP3D.rows() == 3 );
            assert( mP2D.rows() == 2 );
            assert( mP3D.cols() == mP2D.cols() );
            const bool IsRowMajor1 = Derived1::Base::IsRowMajor;
            const bool IsRowMajor2 = Derived2::Base::IsRowMajor;
            EIGEN_STATIC_ASSERT( ( IsRowMajor1 == IsRowMajor2 ), BOTH_MATRICES_MUST_HAVE_THE_SAME_STORAGE_ORDER );
            return project( mP3D.cols(), mP3D.derived().data(), mP2D.derived().data(),
                            !Derived1::Base::IsRowMajor );
        }

        ////////////////////////////////////////////////////////////////////////
        /// Transform points using a rotation and translation and 
        /// project to image
        template<class Derived1,class Derived2, class Derived3, class Derived4>
            bool project_trans( const Eigen::MatrixBase<Derived1>& mR,
                                const Eigen::MatrixBase<Derived3>& mt,
                                const Eigen::MatrixBase<Derived2>& mP3D,
                                Eigen::MatrixBase<Derived4>& mP2D ) const {
            assert( mP3D.rows() == 3 );
            assert( mP2D.rows() == 2 );
            assert( mP3D.cols() == mP2D.cols() );
            assert( mR.rows() == 3 );
            assert( mR.cols() == 3 );
            assert( mt.rows() == 3 );
            assert( mt.cols() == 1 );
            Derived2 mP3DT = mR * mP3D;
            mP3DT.colwise() += mt;
            return project( mP3DT, mP2D );
        }
#endif 
        ////////////////////////////////////////////////////////////////////////
        /// Project points to image and compute the Jacobians
        /// N: number of points
        /// M: number of model parameters
        bool project( const unsigned int nNumPoints, ///<Input: N, the number of points
                      const double* pPoints3D, ///<Input: 3D points
                      double* pPoints2D, ///<Output: 3D points projected into the image 2xN
                      double* pd2d3d,   ///<Output: Jacobian with respect to extrinsics (2x3)xN
                      bool bColMajor = true
                      ) const {
            if( m_pCameraModel != NULL ) {
                return m_pCameraModel->project( nNumPoints,
                                                pPoints3D, pPoints2D,
                                                pd2d3d, bColMajor );
            }
            return false;
        }

        ////////////////////////////////////////////////////////////////////////
        /// Project points to image and compute the Jacobians
        /// N: number of points
        /// M: number of model parameters
        bool project( const unsigned int nNumPoints, ///<Input: N, the number of points
                      const float* pPoints3D, ///<Input: 3D points
                      float* pPoints2D, ///<Output: 3D points projected into the image 2xN
                      float* pd2d3d,   ///<Output: Jacobian with respect to extrinsics (2x3)xN
                      bool bColMajor = true
                      ) const {
            if( m_pCameraModel != NULL ) {
                return m_pCameraModel->project( nNumPoints,
                                                pPoints3D, pPoints2D,
                                                pd2d3d, bColMajor );
            }
            return false;
        }
#if CCAMERAMODEL_HAS_EIGEN3
        ////////////////////////////////////////////////////////////////////////
        /// Project points to image and compute the Jacobians
        template<class Derived1,class Derived2, class Derived3>
            bool project( const Eigen::MatrixBase<Derived1>& mP3D,
                          Eigen::MatrixBase<Derived2>& mP2D,
                          Eigen::MatrixBase<Derived3>& mdP2DE
                          ) const {
            assert( mP3D.rows() == 3 ); assert( mP2D.rows() == 2 );
            assert( mP3D.cols() == mP2D.cols() );
            const bool IsRowMajor1 = Derived1::Base::IsRowMajor;
            const bool IsRowMajor2 = Derived2::Base::IsRowMajor;
            const bool IsRowMajor3 = Derived3::Base::IsRowMajor;
            EIGEN_STATIC_ASSERT( ( IsRowMajor1 == IsRowMajor2 ), BOTH_MATRICES_MUST_HAVE_THE_SAME_STORAGE_ORDER );
            EIGEN_STATIC_ASSERT( ( IsRowMajor1 == IsRowMajor3 ), BOTH_MATRICES_MUST_HAVE_THE_SAME_STORAGE_ORDER );
            if( !project( mP3D.cols(), mP3D.derived().data(), mP2D.derived().data(),
                          mdP2DE.derived().data(), !IsRowMajor1 ) ) {
                return _project_extrinsic( mP3D, mP2D, mdP2DE ); // Generic call
            }
            else {
                return true;
            }
        }
#endif
        ////////////////////////////////////////////////////////////////////////
        /// Project points to image and compute the Jacobians
        /// N: number of points
        /// M: number of model parameters
        bool project( const unsigned int nNumPoints, ///<Input: N, the number of points
                      const double* pPoints3D, ///<Input: 3D points
                      double* pPoints2D, ///<Output: 3D points projected into the image 2xN
                      double* pd2d3d,   ///<Output: Jacobian with respect to extrinsics (2x3)xN
                      double* pd2dParam, ///<Output: Jacobian with respect to intrinsics (2xM)xN
                      bool bColMajor = true
                      ) const {
            if( m_pCameraModel != NULL ) {
                return m_pCameraModel->project( nNumPoints,
                                                pPoints3D, pPoints2D,
                                                pd2d3d, pd2dParam, bColMajor );
            }
            return false;
        }

        ////////////////////////////////////////////////////////////////////////
        /// Project points to image and compute the Jacobians
        /// N: number of points
        /// M: number of model parameters
        bool project( const unsigned int nNumPoints, ///<Input: N, the number of points
                      const float* pPoints3D, ///<Input: 3D points
                      float* pPoints2D, ///<Output: 3D points projected into the image 2xN
                      float* pd2d3d,   ///<Output: Jacobian with respect to extrinsics (2x3)xN
                      float* pd2dParam, ///<Output: Jacobian with respect to intrinsics (2xM)xN
                      bool bColMajor = true
                      ) const {
            if( m_pCameraModel != NULL ) {
                return m_pCameraModel->project( nNumPoints,
                                                pPoints3D, pPoints2D,
                                                pd2d3d, pd2dParam, bColMajor );
            }
            return false;
        }

#if CCAMERAMODEL_HAS_EIGEN3
        ////////////////////////////////////////////////////////////////////////
        /// Project points to image and compute the Jacobians
        template<class Derived1,class Derived2, class Derived3, class Derived4>
            bool project( const Eigen::MatrixBase<Derived1>& mP3D,
                          Eigen::MatrixBase<Derived2>& mP2D,
                          Eigen::MatrixBase<Derived3>& mdP2DE,
                          Eigen::MatrixBase<Derived4>& mdP2DI
                          ) const {
            assert( mP3D.rows() == 3 ); assert( mP2D.rows() == 2 );
            assert( mP3D.cols() == mP2D.cols() );
            const bool IsRowMajor1 = Derived1::Base::IsRowMajor;
            const bool IsRowMajor2 = Derived2::Base::IsRowMajor;
            const bool IsRowMajor3 = Derived3::Base::IsRowMajor;
            const bool IsRowMajor4 = Derived4::Base::IsRowMajor;
            EIGEN_STATIC_ASSERT( ( IsRowMajor1 == IsRowMajor2 ), BOTH_MATRICES_MUST_HAVE_THE_SAME_STORAGE_ORDER );
            EIGEN_STATIC_ASSERT( ( IsRowMajor1 == IsRowMajor3 ), BOTH_MATRICES_MUST_HAVE_THE_SAME_STORAGE_ORDER );
            EIGEN_STATIC_ASSERT( ( IsRowMajor1 == IsRowMajor4 ), BOTH_MATRICES_MUST_HAVE_THE_SAME_STORAGE_ORDER );
            if( !project( mP3D.cols(), mP3D.derived().data(), mP2D.derived().data(),
                          mdP2DE.derived().data(), mdP2DI.derived().data(), !IsRowMajor1 ) ) {
                return _project( mP3D, mP2D, mdP2DE, mdP2DI ); // Generic call
            }
            else {
                return true;
            }
        }

        ////////////////////////////////////////////////////////////////////////////
        template<class Derived1,class Derived2, class Derived3>
            bool _project_extrinsic( const Eigen::MatrixBase<Derived1>& mP3D,
                                     Eigen::MatrixBase<Derived2>& mP2D,
                                     Eigen::MatrixBase<Derived3>& mdP2DE
                                     ) const;

        ////////////////////////////////////////////////////////////////////////////
        template<class Derived1,class Derived2, class Derived3>
            bool _project_intrinsic( const Eigen::MatrixBase<Derived1>& mP3D,
                                     Eigen::MatrixBase<Derived2>& mP2D,
                                     Eigen::MatrixBase<Derived3>& mdP2DI
                                     ) const;

        ////////////////////////////////////////////////////////////////////////
        /// Generic call with finite-difference computation for
        /// the Jacobians (assumes project( mP3D, mP2D ) is implemented).
        template<class Derived1,class Derived2, class Derived3, class Derived4>
            bool _project( const Eigen::MatrixBase<Derived1>& mP3D,
                           Eigen::MatrixBase<Derived2>& mP2D,
                           Eigen::MatrixBase<Derived3>& mdP2DE,
                           Eigen::MatrixBase<Derived4>& mdP2DI
                           ) const;

        ////////////////////////////////////////////////////////////////////////
        /// Transform points using a rotation and translation and 
        /// project to image
        template<class Derived1,class Derived2, class Derived3, 
            class Derived4, class Derived5>
            bool project_trans( const Eigen::MatrixBase<Derived1>& mR,
                                const Eigen::MatrixBase<Derived2>& mt,
                                const Eigen::MatrixBase<Derived3>& mP3D,
                                Eigen::MatrixBase<Derived4>& mP2D,   ////<Output: image coordinates
                                Eigen::MatrixBase<Derived5>& mdP2DE  ///<Output: 2x(6xN) Jacobian
                                ) const;

        ////////////////////////////////////////////////////////////////////////
        /// Transform points using a rotation and translation and 
        /// project to image
        template<class Derived1,class Derived2, class Derived3, 
            class Derived4, class Derived5, class Derived6>
            bool project_trans( const Eigen::MatrixBase<Derived1>& mR,
                                const Eigen::MatrixBase<Derived2>& mt,
                                const Eigen::MatrixBase<Derived3>& mP3D,
                                Eigen::MatrixBase<Derived4>& mP2D,   ////<Output: image coordinates
                                Eigen::MatrixBase<Derived5>& mdP2DE, ///<Output: 2x(6xN) Jacobian
                                Eigen::MatrixBase<Derived6>& mdP2DI  ///<Output: 2x(MxN) Jacobian
                                ) const;
#endif

        ////////////////////////////////////////////////////////////////////////
        /// Transform points from the image plane to an image ray on the unit 
        /// sphere
        bool lift( const unsigned int nNumPoints, 
                   const double* pPoints2D,
                   double* pPoints3D,
                   bool bColMajor = true ) const {
            if( m_pCameraModel != NULL ) {
                return m_pCameraModel->lift( nNumPoints, pPoints2D,
                                             pPoints3D, bColMajor );
            }
            return false;
        }

        ////////////////////////////////////////////////////////////////////////
        /// Transform points from the image plane to an image ray on the unit 
        /// sphere
        bool lift( const unsigned int nNumPoints, 
                   const float* pPoints2D,
                   float* pPoints3D,
                   bool bColMajor = true ) const {
            if( m_pCameraModel != NULL ) {
                return m_pCameraModel->lift( nNumPoints, pPoints2D,
                                             pPoints3D, bColMajor );
            }
            return false;
        }

#if CCAMERAMODEL_HAS_EIGEN3
        ////////////////////////////////////////////////////////////////////////
        /// Lift points to unit sphere
        template<class Derived1, class Derived2>
            bool lift( const Eigen::MatrixBase<Derived1>& mP2D,
                       Eigen::MatrixBase<Derived2>& mP3D
                       ) const {
            assert( mP2D.rows() == 2 ); assert( mP3D.rows() == 3 );
            assert( mP3D.cols() == mP2D.cols() );
            const bool IsRowMajor1 = Derived1::Base::IsRowMajor;
            const bool IsRowMajor2 = Derived2::Base::IsRowMajor;
            EIGEN_STATIC_ASSERT( ( IsRowMajor1 == IsRowMajor2 ), BOTH_MATRICES_MUST_HAVE_THE_SAME_STORAGE_ORDER );
            if( !lift( mP2D.cols(), mP2D.derived().data(), 
                       mP3D.derived().data(), !IsRowMajor1 ) ) {
                return _lift( mP2D, mP3D ); // Generic lift
            }
            return true;
        }
#endif

#if CCAMERAMODEL_HAS_EIGEN3
        ////////////////////////////////////////////////////////////////////////
        /// This function returns a pinhole camera such that no undistorted
        /// pixel will be out of the image.
        ///
        /// It multiplies the focal lengths provided by pvParams by a scale factor
        /// such as the image points will be projected in the distorted image.
        /// if pvParams==NULL, it uses the four first camera parameters as vParams
        CameraModel* new_rectified_camera
            ( const int nImageWidth, 
              const int nImageHeight,
              const std::vector<double>* pvParams = NULL ///< Input: fx, fy, cx, cy
              );

        ////////////////////////////////////////////////////////////////////////
        void undistort_image( const int nImageWidth,
                              const int nImageHeight,
                              const int nImageWidthStep,
                              const unsigned char* pImageIn,
                              unsigned char* pImageOut,
                              bool bRecompute = false ); 

#  if CCAMERAMODEL_HAS_OPENCV
        ////////////////////////////////////////////////////////////////////////
        void undistort_image( const IplImage* pImageIn,
                              IplImage* pImageOut,
                              bool bRecompute = false ); 
#  endif
#endif

        ////////////////////////////////////////////////////////////////////////
        bool initialise_parameters( const int nImageWidth, const int nImageHeight ) {
            if( m_pCameraModel != NULL ) { 
                return m_pCameraModel->initialise_parameters( nImageWidth, nImageHeight );
            }
            return false;
        }

        ////////////////////////////////////////////////////////////////////////
        int get_num_parameters() const {
            if( m_pCameraModel != NULL ) { 
                return m_pCameraModel->get_num_parameters();
            }
            return -1;
        }

        ////////////////////////////////////////////////////////////////////////
        int name_to_parameter_index( const std::string& sName ) const {
            if( m_pCameraModel != NULL ) { 
                return m_pCameraModel->name_to_parameter_index( sName );
            }
            return -1;
        }

        ////////////////////////////////////////////////////////////////////////
        std::string parameter_index_to_name( const int nIndex ) const {
            if( m_pCameraModel != NULL ) { 
                return m_pCameraModel->parameter_index_to_name( nIndex );
            }
            return "";
        }

#if CCAMERAMODEL_HAS_EIGEN3
        ////////////////////////////////////////////////////////////////////////
        template <class Derived>
             bool set_parameters( const Eigen::MatrixBase<Derived>& vParameters
                                  ) {
            for( int ii=0; ii<vParameters.rows(); ii++ ) {
                std::string sName = parameter_index_to_name( ii );
                if( sName == "" ) { return false; }
                set( sName, vParameters(ii,0) );
            }
            return true;
        }
        
        ////////////////////////////////////////////////////////////////////////
        template <class Derived>
             bool get_parameters( Eigen::MatrixBase<Derived>& vParameters
                                  ) const { 
            for( int ii=0; ii<vParameters.rows(); ii++ ) {
                std::string sName = parameter_index_to_name( ii );
                if( sName == "" ) { return false; }
                vParameters(ii,0) = get<typename Eigen::internal::traits<Derived>::Scalar>( sName );
            }
            return true;
        }
#endif

        ////////////////////////////////////////////////////////////////////////
        /// Give information about the current camera
        void info() const {
            if( m_pCameraModel != NULL ) {
                m_pCameraModel->info();
            }
        }

        ////////////////////////////////////////////////////////////////////////
        /// Generic get (does not appear in the interface)
        template <class T>
            T get( const std::string& sKey ) const {
            if( m_pCameraModel == NULL ) {
                return false;
            }
            return FromString<T>( m_pCameraModel->get( sKey ) ); 
        }

        ////////////////////////////////////////////////////////////////////////
        ///
        std::string get( const std::string& sKey ) const {
            if( m_pCameraModel == NULL ) {
                return "";
            }
            return m_pCameraModel->get( sKey );
        }

        ////////////////////////////////////////////////////////////////////////
        /// Generic set (does not appear in the interface)
        template <class T>
            bool set( const std::string& sKey, const T& parameter ) {
            if( m_pCameraModel == NULL ) {
                return false;
            }
            const std::string sParameter = ToString<T>( parameter );
            return m_pCameraModel->set( sKey, sParameter );
        }

        ////////////////////////////////////////////////////////////////////////
        ///
        bool set( const std::string& sKey, const std::string& sParameter ) {
            if( m_pCameraModel == NULL ) {
                return false;
            }
            return m_pCameraModel->set( sKey, sParameter );
        }

        ////////////////////////////////////////////////////////////////////////
        bool save( const std::string& sFileName );

        ////////////////////////////////////////////////////////////////////////
        bool load( const std::string& sFileName );

        ////////////////////////////////////////////////////////////////////////
    private:
        CameraModelInterface<>* m_pCameraModel;
        Eigen::MatrixXf m_mLUT;

    private:    
#if CCAMERAMODEL_HAS_EIGEN3
        ////////////////////////////////////////////////////////////////////////
        /// Lift points to unit sphere
        /// (generic function, uses Gauss-Newton estimation)
        template<class Derived1, class Derived2>
            bool _lift( const Eigen::MatrixBase<Derived1>& mP2D,
                        Eigen::MatrixBase<Derived2>& mP3D ) const;
#endif

#if CCAMERAMODEL_HAS_EIGEN3
        ////////////////////////////////////////////////////////////////////////
        CameraModel* _new_rectified_camera
            ( const int nImageWidth, 
              const int nImageHeight,
              const std::vector<double>* pvParams = NULL
              );
#endif

        ////////////////////////////////////////////////////////////////////////
        void ToStream( std::ostream& oStream ) const {
            if( m_pCameraModel == NULL ) {
                return;
            }
            oStream << *m_pCameraModel;
        }

        ////////////////////////////////////////////////////////////////////////
        void FromStream( std::istream& iStream ) {
            if( m_pCameraModel == NULL ) {
                return;
            }
            iStream >> *m_pCameraModel;
        }
    };
}

#endif
