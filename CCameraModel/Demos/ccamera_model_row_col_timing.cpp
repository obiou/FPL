#include <iostream>
#include <string>

#include <CameraModel.h>
#include <Misc.h>

#include <eigen3/Eigen/Core>

using namespace CCameraModel;
using namespace Eigen;
using namespace std;

typedef Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic,Eigen::RowMajor> MatrixXdRowMajor;
typedef Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic,Eigen::ColMajor> MatrixXdColMajor;
typedef Eigen::Matrix<double,Eigen::Dynamic,1,Eigen::RowMajor> VectorXdRowMajor;
typedef Eigen::Matrix<double,Eigen::Dynamic,1,Eigen::ColMajor> VectorXdColMajor;

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

    const int nNumPoints = 1000000;

    MatrixXdColMajor mP3DColMajor = MatrixXdColMajor::Random(3,nNumPoints);
    //MatrixXdRowMajor mP3DRowMajor = MatrixXdRowMajor::Random(3,nNumPoints);
    MatrixXdRowMajor mP3DRowMajor = mP3DColMajor;

    MatrixXdColMajor mP2DColMajor(2,nNumPoints);
    MatrixXdRowMajor mP2DRowMajor(2,nNumPoints);
 
    MatrixXdColMajor mdP2DEColMajor( 2, 3*nNumPoints );
    MatrixXdColMajor mdP2DIColMajor( 2, cameraModel.get_num_parameters()*nNumPoints );

    MatrixXdRowMajor mdP2DERowMajor( 2, 3*nNumPoints );
    MatrixXdRowMajor mdP2DIRowMajor( 2, cameraModel.get_num_parameters()*nNumPoints );

    double t0 = CMISC::Tic();
    if( !cameraModel.project( mP3DColMajor, mP2DColMajor, mdP2DEColMajor, mdP2DIColMajor ) ) {
        return -1;
    }
    cout << "Time taken to project (col_major): " << CMISC::TocMS( t0 ) << endl;

    t0 = CMISC::Tic();
    if( !cameraModel.project( mP3DRowMajor, mP2DRowMajor, mdP2DERowMajor, mdP2DIRowMajor ) ) {
        return -1;
    }
    cout << "Time taken to project (row_major): " << CMISC::TocMS( t0 ) << endl;
}
