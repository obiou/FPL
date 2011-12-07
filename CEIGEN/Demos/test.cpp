#include <stdio.h>

#include <iostream>

#include <ceigen.h>
#include <Misc.h>

using namespace Eigen;
using namespace std;

using namespace CEIGEN;
using namespace CMISC;

const int NUM_ITERS = 5000;

////////////////////////////////////////////////////////////////////////////
int main() {
    MatrixXf mSkew = skew_rot<MatrixXf,double>( 1, 2, 3 );
    cout << "mSkew: " << endl << mSkew << endl;

    cout << "mSkew: " << endl << skew_rot<MatrixXf>( 1, 2, 3 ) << endl;

    Vector3d v; v << 1, 0, 0;
    cout << "mSkew: " << endl << skew_rot<MatrixXf,Vector3d>( v ) << endl;

    MatrixXf m = MatrixXf::Random(3,10);
    MatrixXi mR = MatrixXi::Random(1,1);
    cout << "m =" << endl << m << endl;

    cout << "metric( m ): " << endl << metric( m ) << endl;
    cout << "metric_slow( m ): " << endl << metric_slow( m ) << endl;

    int nSum = 0; // to avoid optimisation
    double t0, t1;
    double dTime0 = 0, dTime1 = 0;
    MatrixXf m0, m1;
    for( int ii=0; ii<NUM_ITERS; ii++ ) {
        m = MatrixXf::Random( 3, 10 );
        t0 = Tic();
        m0 = metric( m );
        dTime0 += TocMS( t0 );
        nSum += m0.row( 0 ).sum();

        t1 = Tic();
        m1 = metric_slow( m );
        dTime1 += TocMS( t1 );
        nSum += m1.row( 0 ).sum();
    }
    cout << "sum: " << nSum << endl;
    cout << "Time0 metric: " << dTime0 << endl;
    cout << "Time1 metric_slow: " << dTime1 << endl;

    dTime0 = 0; dTime1 = 0; 
    double t2 = 0;
    double dTime2 = 0;
    double dSum = 0;
    const int nV = 300;
    for( int ii=0; ii<NUM_ITERS; ii++ ) {
        mR = MatrixXi::Random(1,1);
        m = MatrixXf::Random( 3, nV );
        t0 = Tic();
        dSum += sqrt( m.colwise().squaredNorm().mean() );
        dTime0 += TocMS( t0 );

        t1 = Tic();
        dSum += m.lpNorm<2>() / sqrt( mR(1,1) );
        dTime1 += TocMS( t1 );

        t2 = Tic();
        dSum += sqrt( m.squaredNorm() / mR(1,1) );
        dTime2 += TocMS( t2 );

        dSum /= nV*nV;
    }
    cout << "sum: " << dSum << endl;
    cout << "Time colwise: " << dTime0 << endl;
    cout << "Time lpnorm: "  << dTime1 << endl;
    cout << "Time squaredNorm: "  << dTime2 << endl;

    cout << sqrt( m.squaredNorm() / mR(1,1) ) << endl;
    cout << m.lpNorm<2>() / sqrt( mR(1,1) ) << endl;
}
