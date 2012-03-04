#include <cgeom/CGeom.h>
#include <ceigen.h>

#include <fstream>
#include <iostream>
#include <queue>
#include <sstream>
#include <string>

#include <Eigen/Core>
#include <Eigen/LU>
#include <Eigen/Cholesky>

#include <eigen3/Eigen/Cholesky>
#include <Eigen/LU>
#include <eigen3/Eigen/SVD>

using namespace Eigen;
using namespace std;

using namespace CEIGEN;

////////////////////////////////////////////////////////////////////////////////
int main() {
    MatrixXd mP3D;
    MatrixXd mM2D;
    fstream fs1( "p3d.txt", fstream::in );
    if( fs1.good() ) {
        fs1 >> mP3D;
    }
    else {
        return -1;
    }
    fstream fs2( "m2d.txt", fstream::in );
    if( fs2.good() ) {
        fs2 >> mM2D;
    }
    else {
        return -1;
    }
    if( mM2D.rows() == 2 ) {
        mM2D = CEIGEN::projective( mM2D );
    }

    if( mM2D.rows() != 3 ) {
        cerr << "Wrong number of rows, expecting 3, got: " << mM2D.rows() << endl;
        return -1;
    }

    //cout << "mP3D: " << endl << mP3D << endl;
    //cout << "mM2D: " << endl << mM2D << endl;

    const int nMaxNumIters = 30;
    const double dTol      = 1e-5;
    const double dEpsilon  = 1e-8;
    Eigen::Matrix3d mR = Eigen::Matrix3d::Identity();
    Eigen::Vector3d vt;
    int nNumIterations = 0;
    double dObjError;
    CGEOM::objpose( mP3D, mM2D,
                    nMaxNumIters, dTol, dEpsilon,
                    mR, vt, nNumIterations, dObjError, false );
    cout << mR << endl;
    cout << vt << endl;
    cout << "nNumIterations: " << nNumIterations << endl;
    cout << "dObjError: " << dObjError << endl;

    return (dObjError < 0.1) ? 0 : -1;
}
