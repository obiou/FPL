#include <math.h>
#include <stdio.h>

#include <algorithm>
#include <iostream>
#include <iterator>
#include <map>
#include <string>

#include <CameraModel.h>
#include <CCameraModel/ProjectionFunctions.h>
#include <ceigen.h>
#include <COpenCV.h>
#include <Misc.h>

#include <cv.h>
#include <highgui.h>

//#define EIGEN_DEFAULT_TO_ROW_MAJOR 1
#include <eigen3/Eigen/Core>

using namespace CCameraModel;
using namespace CPROJECTIONS;
using namespace Eigen;
using namespace std;

//////////////////////////////////////////////////////////////////////////////// 
void apply_update( const Eigen::VectorXd& vUpdate,
                   Eigen::Matrix3d& mR, Eigen::Vector3d& mt ) {
    mR *= ( Eigen::Matrix3d::Identity() + 
            CEIGEN::skew_rot<Eigen::Matrix3d,double>(vUpdate[0],0.,0.) +
            CEIGEN::skew_rot<Eigen::Matrix3d,double>(0.,vUpdate[1],0.) +
            CEIGEN::skew_rot<Eigen::Matrix3d,double>(0.,0.,vUpdate[2]) );
    mt(0,0) += vUpdate[3];
    mt(1,0) += vUpdate[4];
    mt(2,0) += vUpdate[5];
}

////////////////////////////////////////////////////////////////////////////////
int main()
{
    double fx = 200.; double fy = 130.; double cx = 210.; double cy = 300.;
    //double k1 = 2; double k2 = -1; double p1 = 4; double p2 = 0.3;
    double k1 = -0.8; double k2 = 0.6; double p1 = 0; double p2 = 0;

    CameraModel cameraModel( "PinholeRadTan" );
    //CameraModel cameraModel( "Pinhole" );
    cameraModel.set( "fx", fx );
    cameraModel.set( "fy", fy );
    cameraModel.set( "cx", cx );
    cameraModel.set( "cy", cy );
    cameraModel.set( "k1", k1 );
    cameraModel.set( "k2", k2 );
    cameraModel.set( "p1", p1 );
    cameraModel.set( "p2", p2 );

    CvMat* rot_vect          = cvCreateMat( 1, 3, CV_64FC1 );
    CvMat* trans_vect        = cvCreateMat( 1, 3, CV_64FC1 );
    CvMat* intrinsic_matrix  = cvCreateMat( 3, 3, CV_64FC1 );
    CvMat* distortion_coeffs = cvCreateMat( 4, 1, CV_64FC1 );
    cvZero( rot_vect );
    cvZero( trans_vect );
    cvZero( distortion_coeffs );

    cvZero( intrinsic_matrix );
    cvmSet( intrinsic_matrix, 0, 0, fx );
    cvmSet( intrinsic_matrix, 1, 1, fy );
    cvmSet( intrinsic_matrix, 0, 2, cx );
    cvmSet( intrinsic_matrix, 1, 2, cy );
    cvmSet( intrinsic_matrix, 2, 2, 1 );

    cvmSet( distortion_coeffs, 0, 0, k1 );
    cvmSet( distortion_coeffs, 1, 0, k2 );
    cvmSet( distortion_coeffs, 2, 0, p1 );
    cvmSet( distortion_coeffs, 3, 0, p2 );

    // Give some 'random' values
    cvmSet( trans_vect, 0, 0, 10 );
    cvmSet( trans_vect, 0, 1, -2 );
    cvmSet( trans_vect, 0, 2, 3 );
    cvmSet( rot_vect, 0, 0, 0.5);
    cvmSet( rot_vect, 0, 1, -.3 );
    cvmSet( rot_vect, 0, 2, 1 );

    CvMat* rot_mat = cvCreateMat( 3, 3, CV_64FC1 );
    cvRodrigues2( rot_vect, rot_mat );

    const int nNumPoints = 4; // MUST BE BIGGER THAN 4!!!

    CvMat* pIn  = cvCreateMat( 3, nNumPoints, CV_64FC1 );
    CvMat* pOut = cvCreateMat( 2, nNumPoints, CV_64FC1 );

    // Fill matrices
    for( int ii=0; ii<nNumPoints; ii++ ) {
        cvmSet( pIn, 0, ii, random()/100 );
        cvmSet( pIn, 1, ii, random()/100 );
        cvmSet( pIn, 2, ii, random()/1000 );
    }

    //COPENCV::print( pIn );

    CvMat* pOutR = cvCreateMat( 2*nNumPoints, 3, CV_64FC1 );
    CvMat* pOutt = cvCreateMat( 2*nNumPoints, 3, CV_64FC1 );
    CvMat* pOutF = cvCreateMat( 2*nNumPoints, 2, CV_64FC1 );
    CvMat* pOutC = cvCreateMat( 2*nNumPoints, 2, CV_64FC1 );
    CvMat* pOutD = cvCreateMat( 2*nNumPoints, 4, CV_64FC1 );

    cvProjectPoints2( pIn, rot_vect, trans_vect,
                      intrinsic_matrix, distortion_coeffs, pOut,
                      pOutR, pOutt, pOutF, pOutC, pOutD );


    MatrixXd mP3D(3,nNumPoints);
    CEIGEN::copy( *pIn, mP3D );
    MatrixXd mP2D(2,nNumPoints);
    Matrix3d mR = Matrix3d::Identity();
    CEIGEN::copy( *rot_mat, mR );    
    Vector3d mt;
    CEIGEN::copy( *trans_vect, mt );
    
#if 0
    MatrixXd mdP2DE( 2, 3*nNumPoints );
    MatrixXd mdP2DI( 2, cameraModel.get_num_parameters()*nNumPoints );
    if( !cameraModel.project( mP3D, mP2D, mdP2DE, mdP2DI ) ) {
        return -1;
    }
#else
    MatrixXd mdP2DE( 2, 6*nNumPoints );
    MatrixXd mdP2DI( 2, cameraModel.get_num_parameters()*nNumPoints );
    if( !cameraModel.project_trans( mR, mt, mP3D, mP2D, mdP2DE, mdP2DI ) ) {
        return -1;
    }
#endif

    cout << "pIn:" << endl;
    COPENCV::print( pIn );
    cout << "mP3D: " << endl << mP3D << endl << endl;

    cout << "pOut:" << endl;
    COPENCV::print( pOut );
    cout << "mP2D: " << endl << mP2D << endl << endl;

    cout << "pOutR:" << endl;
    COPENCV::print( pOutR );
    cout << "pOutt:" << endl;
    COPENCV::print( pOutt );
    cout << "mdP2DE: " << endl << mdP2DE << endl << endl;

    Eigen::MatrixXd mdP2Dfd( 2, nNumPoints );
    Eigen::MatrixXd mP2Dfd( 2, nNumPoints );
    float fEps = 0.01;
    Eigen::Matrix3d mReps;
    Eigen::Vector3d mteps;
    const unsigned int nNumPoseParams = 6;
    for( size_t kk=0; kk<nNumPoseParams; kk++ ) {
        Eigen::Matrix<double,6,1> vEps = Eigen::Matrix<double,nNumPoseParams,1>::Zero();
        vEps(kk) = fEps;
        mReps = mR;
        mteps = mt;
        apply_update( vEps, mReps, mteps );
        cameraModel.project_trans( mReps, mteps, mP3D, mP2Dfd );
        mdP2Dfd = ( mP2Dfd - mP2D ) / fEps;
        for( size_t ll=0; ll<(unsigned int)nNumPoints; ll++ ) {
            mdP2DE.col( nNumPoseParams*ll + kk ) = mdP2Dfd.col( ll );
        }
    }
    cout << "mdP2DEfd: " << endl << mdP2DE << endl << endl;


    cout << "pOutF:" << endl;
    COPENCV::print( pOutF );
    cout << "pOutC:" << endl;
    COPENCV::print( pOutC );
    cout << "pOutD:" << endl;
    COPENCV::print( pOutD );
    cout << "mdP2DI: " << endl << mdP2DI << endl;

    cout << endl;
    
    cout  << endl << "Lifting test:" << endl;
    
    MatrixXd mP3DEst( 3, nNumPoints );
    cameraModel.lift( mP2D, mP3DEst );

    cout << "mP3DEst:" << endl << mP3DEst << endl;

    Eigen::MatrixXd mP3DT = mR * mP3D;
    mP3DT.colwise() += mt;

    for( int nIndex=0; nIndex<nNumPoints; nIndex++ ) {
        mP3DT.col( nIndex ) /= mP3DT.col( nIndex ).norm();
    }
    cout << "mP3DT:" << endl << mP3DT << endl;

    return 0;
#if 0
        
        printf( "O %f %f \n", 
                CV_MAT_ELEM( *pOut, double, 0, 0 ),
                CV_MAT_ELEM( *pOut, double, 0, 1 ) );
        
        printf( "%f %f %f %f %f %f / %f %f %f %f / %f %f %f %f / %f %f %f %f / %f %f %f %f\n\n", 
                CV_MAT_ELEM( *pOutt, double, 0, 0 ),
                CV_MAT_ELEM( *pOutt, double, 1, 0 ),
                CV_MAT_ELEM( *pOutt, double, 0, 1 ),
                CV_MAT_ELEM( *pOutt, double, 1, 1 ),
                CV_MAT_ELEM( *pOutt, double, 0, 2 ),
                CV_MAT_ELEM( *pOutt, double, 1, 2 ),
                CV_MAT_ELEM( *pOutF, double, 0, 0 ),
                CV_MAT_ELEM( *pOutF, double, 1, 0 ),
                CV_MAT_ELEM( *pOutF, double, 0, 1 ),
                CV_MAT_ELEM( *pOutF, double, 1, 1 ),
                CV_MAT_ELEM( *pOutC, double, 0, 0 ),
                CV_MAT_ELEM( *pOutC, double, 1, 0 ),
                CV_MAT_ELEM( *pOutC, double, 0, 1 ),
                CV_MAT_ELEM( *pOutC, double, 1, 1 ),
                CV_MAT_ELEM( *pOutD, double, 0, 0 ),
                CV_MAT_ELEM( *pOutD, double, 1, 0 ),
                CV_MAT_ELEM( *pOutD, double, 0, 1 ),
                CV_MAT_ELEM( *pOutD, double, 1, 1 ),
                CV_MAT_ELEM( *pOutD, double, 0, 2 ),
                CV_MAT_ELEM( *pOutD, double, 1, 2 ),
                CV_MAT_ELEM( *pOutD, double, 0, 3 ),
                CV_MAT_ELEM( *pOutD, double, 1, 3 )
                );
	}
#endif 
}
