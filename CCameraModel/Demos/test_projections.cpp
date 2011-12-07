#include <math.h>
#include <stdio.h>

#include <algorithm>
#include <iostream>
#include <iterator>
#include <map>
#include <string>

#include <CameraModel.h>
#include <CCameraModel/ProjectionFunctions.h>
#include <COpenCV.h>
#include <Misc.h>

#include <cv.h>
#include <highgui.h>

//#define EIGEN_DEFAULT_TO_ROW_MAJOR 1
#include <eigen3/Eigen/Core>

using namespace CPROJECTIONS;
using namespace Eigen;
using namespace std;

#define ARRAY_SIZE 1024
#define TEST_ITERATIONS 2000
 
////////////////////////////////////////////////////////////////////////////////
template <class T>
void InitWithRandom( T *ptr, int num )
{
	while( num > 0 ) {
		*ptr = (T)random()/5;
		++ptr; --num;
	}
}

using namespace CCameraModel;

////////////////////////////////////////////////////////////////////////////////
template <class T>
void projectCmp( T* in1, T*  in2, T*  in3, T* out1, T* out2, const int num )
{
    double fx = 2.; double fy = 1.3; double cx = 2.; double cy = 10.;
    double k1 = 2; double k2 = 0; double p1 = 4; double p2 = 0.3;

    CameraModel cameraModel( "PinholeRadTan" );
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

#if 1
    cvmSet( distortion_coeffs, 0, 0, k1 );
    cvmSet( distortion_coeffs, 1, 0, k2 );
    cvmSet( distortion_coeffs, 2, 0, p1 );
    cvmSet( distortion_coeffs, 3, 0, p2 );
#endif

    int nNumPoints = 3;

    CvMat* pIn  = cvCreateMat( 3, nNumPoints, CV_64FC1 );
    CvMat* pOut = cvCreateMat( 2, nNumPoints, CV_64FC1 );

    CvMat* pOutP = cvCreateMat( 2*nNumPoints, 3, CV_64FC1 );
    CvMat* pOutF = cvCreateMat( 2*nNumPoints, 2, CV_64FC1 );
    CvMat* pOutC = cvCreateMat( 2*nNumPoints, 2, CV_64FC1 );
    CvMat* pOutD = cvCreateMat( 2*nNumPoints, 4, CV_64FC1 );

	for ( int i = 0; i < num ; ++i ) {
        double divz = 1/in3[i];
        double xz = in1[i]*divz;
        double yz = in2[i]*divz;
        out1[i] = fx * xz + cx;
        out2[i] = fy * yz + cy;

#if 0
        //printf( "%f %f\n", out1[i], out2[i] );
        double diffx[3], diffy[3]; 
        double dFx[2]; double dFy[2];
        double dCx[2]; double dCy[2];
        double dk1[2]; double dk2[2];
        double dp1[2]; double dp2[2];
        double dummy[2];
        CPROJECTIONS::project<double,1,1,1,1,1,1,0>
            ( fx, fy, cx, cy, k1, k2, p1, p2, in1[i], in2[i], in3[i],
              &out1[i], &out2[i],
              diffx, diffy, dFx, dFy, dCx, dCy,
              dk1, dk2, dp1, dp2
              );
        printf( "P %f %f \n", out1[i], out2[i] );
        printf( "%f %f %f %f %f %f / %f %f %f %f / %f %f %f %f / %f %f %f %f / %f %f %f %f\n\n",
                diffx[0], diffy[0], diffx[1], diffy[1], diffx[2], diffy[2],
                dFx[0], dFx[1], dFy[0], dFy[1],
                dCx[0], dCx[1], dCy[0], dCy[1],
                dk1[0], dk1[1], dk2[0], dk2[1],
                dp1[0], dp1[1], dp2[0], dp2[1]
                );
#endif

#if 0
        double dEPS = 1e-6; double d1, d2;
        CPROJECTIONS::project<double,1,1,1,1,1,1,0>
            ( fx, fy, cx, cy, k1, k2, p1, p2, in1[i], in2[i], in3[i]+dEPS,
              &d1, &d2,
              diffx, diffy,
              dummy, dummy, dummy, dummy,
              dummy, dummy, dummy, dummy
              );  
        //printf( "E: %f %f\n", (d1 - out1[i])/dEPS, (d2 - out2[i])/dEPS);
#endif

        cvmSet( pIn, 0, 0, in1[i] );
        cvmSet( pIn, 0, 1, in2[i] );
        cvmSet( pIn, 0, 2, in3[i] );

        cvProjectPoints2( pIn, rot_vect, trans_vect,
                          intrinsic_matrix, distortion_coeffs, pOut,
                          NULL, pOutP, pOutF, pOutC, pOutD );

        printf( "O %f %f \n", 
                CV_MAT_ELEM( *pOut, double, 0, 0 ),
                CV_MAT_ELEM( *pOut, double, 0, 1 ) );
        
        printf( "%f %f %f %f %f %f / %f %f %f %f / %f %f %f %f / %f %f %f %f / %f %f %f %f\n\n", 
                CV_MAT_ELEM( *pOutP, double, 0, 0 ),
                CV_MAT_ELEM( *pOutP, double, 1, 0 ),
                CV_MAT_ELEM( *pOutP, double, 0, 1 ),
                CV_MAT_ELEM( *pOutP, double, 1, 1 ),
                CV_MAT_ELEM( *pOutP, double, 0, 2 ),
                CV_MAT_ELEM( *pOutP, double, 1, 2 ),
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

        T in[3]; T out[2];
        in[0] = in1[i]; in[1] = in2[i]; in[2] = in3[i];
#if 0
        cameraModel.project( 1, in, out );
        printf( ">>> C %f %f (in %f, %f, %f)\n", out[0], out[1],
                in[0], in[1], in[2] );
#endif
        T pdOutE[2*3], pdOutI[2*(4+4)];
        cameraModel.project( 1, in, out, pdOutE, pdOutI );
        printf( ">>> C %f %f\n%f %f %f %f %f %f / %f %f %f %f / %f %f %f %f / %f %f %f %f / %f %f %f %f )\n", 
                out[0], out[1],
                pdOutE[0], pdOutE[3], pdOutE[1], pdOutE[4], pdOutE[2], pdOutE[5],
                pdOutI[0], pdOutI[1], pdOutI[2], pdOutI[3], pdOutI[4], pdOutI[5], pdOutI[6], pdOutI[7],
                pdOutI[8], pdOutI[9], pdOutI[10], pdOutI[11], pdOutI[12], pdOutI[13], pdOutI[14], pdOutI[15]
                );

#if 1
        nNumPoints = 1;
        MatrixXd mP3D(3,nNumPoints);
        mP3D = Map<MatrixXd>( in, 3, nNumPoints );
        MatrixXd mP2D(2,nNumPoints);
        Matrix3d mR = Matrix3d::Identity();
        Vector3d mt = Vector3d::Zero();

        MatrixXd mdP2DE( 2, 3*nNumPoints );
        MatrixXd mdP2DI( 2, cameraModel.get_num_parameters()*nNumPoints );

        cameraModel.project( mP3D, mP2D, mdP2DE, mdP2DI );
        //cout << "mP3D: " << endl << mP3D << endl;
        cout << "mP2D: " << endl << mP2D << endl;
        cout << "mdP2DE: " << endl << mdP2DE << endl;
        cout << "mdP2DI: " << endl << mdP2DI << endl;
#endif
        cout << "pOutP:" << endl;
        COPENCV::print( pOutP );
        cout << "pOutF:" << endl;
        COPENCV::print( pOutF );
        cout << "pOutC:" << endl;
        COPENCV::print( pOutC );
        cout << "pOutD:" << endl;
        COPENCV::print( pOutD );

        cout << endl;
	}

    cvReleaseMat( &rot_vect );
    cvReleaseMat( &trans_vect );
    cvReleaseMat( &intrinsic_matrix );
    cvReleaseMat( &distortion_coeffs );
}

////////////////////////////////////////////////////////////////////////////////
template <class T>
void projectTest( T* in1, T*  in2, T*  in3, T* out1, T* out2, const int num )
{
    double fx = 2.; double fy = 1.3; double cx = 2.; double cy = 10.;
	for ( int i = 0; i < num ; ++i ) {
        double divz = 1/in3[i];
        double xz = in1[i]*divz;
        double yz = in2[i]*divz;
        out1[i] = fx * xz + cx;
        out2[i] = fy * yz + cy;

	}
}

////////////////////////////////////////////////////////////////////////////////
template <class T>
void projectTest2( T* in1, T*  in2, T*  in3, T* out1, T* out2, const int num )
{
    double fx = 2.; double fy = 1.3; double cx = 2.; double cy = 10.;
    double k1 = 2; double k2 = 0; double p1 = 4; double p2 = 0.3;
	for ( int i = 0; i < num ; ++i ) {
        CPROJECTIONS::project<double,0,0,0,0,0>
            ( fx, fy, cx, cy, k1, k2, p1, p2, in1[i], in2[i], in3[i],
              &out1[i], &out2[i] );
	}
}

////////////////////////////////////////////////////////////////////////////////
int main()
{
    double A[ ARRAY_SIZE ];
	double B[ ARRAY_SIZE ];
	double C[ ARRAY_SIZE ];
	double D[ ARRAY_SIZE ];
	double E[ ARRAY_SIZE ];
 
	InitWithRandom( A , ARRAY_SIZE );
    InitWithRandom( B , ARRAY_SIZE );
	InitWithRandom( C , ARRAY_SIZE );

    projectCmp( A, B, C, D, E, ARRAY_SIZE );
 
	double dRetVal = 0;
	double dBeg, dTotal;
	int dontOptimizeThisLoopToNothing = 0;
	for ( int i = 0 ; i < TEST_ITERATIONS ; ++i ) {
		dBeg = CMISC::Tic();
	
        projectTest( A, B, C, D, E, ARRAY_SIZE );

		dTotal = CMISC::TocMS( dBeg );
		dontOptimizeThisLoopToNothing  += i;
		dRetVal += dTotal;
	}
	// force compiler to actually use the data so it doesn't vanish the loop above
	float ac = 0;
	for ( int i = 0 ; i < ARRAY_SIZE ; ++i ) {
		ac += D[i] + E[i];
	}
	printf( "%f %d\n", ac, dontOptimizeThisLoopToNothing  ); 
    printf( "timing:   %.3f ms\n", dRetVal );

    dRetVal = 0;
	dontOptimizeThisLoopToNothing = 0;
	for ( int i = 0 ; i < TEST_ITERATIONS ; ++i ) {
		dBeg = CMISC::Tic();
	
        projectTest2( A, B, C, D, E, ARRAY_SIZE );

		dTotal = CMISC::TocMS( dBeg );
		dontOptimizeThisLoopToNothing  += i;
		dRetVal += dTotal;
	}
	// force compiler to actually use the data so it doesn't vanish the loop above
	ac = 0;
	for ( int i = 0 ; i < ARRAY_SIZE ; ++i ) {
		ac += D[i] + E[i];
	}

    printf( "timing:   %.3f ms\n", dRetVal );

    return 0;
}
