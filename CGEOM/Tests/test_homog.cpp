#include <cgeom/CGeom.h>
#include <ceigen.h>

#include <iostream>

#include <Eigen/Core>
#include <Eigen/LU>
#include <Eigen/Cholesky>

using namespace Eigen;
using namespace std;

int main() {
    Matrix<double,2,4> mC1 = Matrix<double,2,4>::Random();
    MatrixXd mC2 = MatrixXd::Random(2,4);
    double dBoxSize = 10;//Matrix<double,1,1>::Random()(0,0);

    cout << "mC1: " << endl << mC1 << endl;
    cout << "mC2: " << endl << mC2 << endl;

    Matrix3d mHBox = CGEOM::ComputeHomography4Points( mC1, dBoxSize );
    MatrixXd m = mHBox.inverse()*CEIGEN::projective( mC1 );
    cout << "mHBox*mC1 (box of size 10): " << endl << CEIGEN::metric( m ) << endl;

    Matrix3d mH = CGEOM::ComputeHomography4Points( mC1, mC2 );
    cout << "mC2 - H*mC1 (should be 0): " << endl << mC2 - CEIGEN::metric( mH*CEIGEN::projective( mC1 ) ) << endl;
    
    double dRes =  (mC2 - CEIGEN::metric( mH*CEIGEN::projective( mC1 ) )).array().abs().sum();
    cout << dRes << endl;

    return (dRes < 1e-10) ? 0 : -1;
}
