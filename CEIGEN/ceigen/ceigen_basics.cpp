#include <ceigen/ceigen_basics.h>

#include <numeric>

using namespace std;

////////////////////////////////////////////////////////////////////////////////
double CEIGEN::median( const Eigen::VectorXd& vSqNormE ) {
    assert( vSqNormE.rows() != 0 );
    std::vector<double> d( vSqNormE.data(), vSqNormE.data()+vSqNormE.rows() );
    nth_element( d.begin(), d.begin() + (d.size() >> 1), d.end() );
    return d[ d.size() >> 1 ];
}

////////////////////////////////////////////////////////////////////////////////
static const double kdHuberThreshold = 1.2107;
static const double kdHuberThresholdSq = kdHuberThreshold*kdHuberThreshold;

////////////////////////////////////////////////////////////////////////////////
static inline double sq( const double d ) { return d*d; }

////////////////////////////////////////////////////////////////////////////////
Eigen::VectorXd CEIGEN::compute_weights( const Eigen::VectorXd& vR,
                                         const int nNumParams,
                                         const double dMADFact,
                                         std::vector<bool>& vInliers ) {
    const int nNumResiduals = vR.rows();
    const double dNumResiduals = vR.rows();
    vInliers.resize( nNumResiduals );

    Eigen::VectorXd mW( nNumResiduals );
    // Compute: abs( vR - median(vR) )
    const double dMedian = median( vR );
    Eigen::VectorXd vRCentered = vR; vRCentered.array() -= dMedian;
    vRCentered = vRCentered.array().abs();
    // Compute robust standard deviation (MAD)
    const double dMAD  = 
        1.4826 * (1.+5./(dNumResiduals-double(nNumParams))) *
        median( vRCentered );

    //cout << "MAD: " << dMAD << endl;
    const int nMinNumInliers = 8;

    if( true ) {
        double dC = 4.6851 * dMAD * dMADFact;
        double dMADFactAdapt = dMADFact;
        int nNumInliers = 0;

        while( nNumInliers < nMinNumInliers ) {
            //cout << "dMADFactAdapt: " << dMADFactAdapt << endl;
            // Tukey weights
            dC = 4.6851 * dMAD * dMADFactAdapt;
            for( int ii=0; ii<nNumResiduals; ii++ ) {
                vInliers[ii] = vRCentered[ii] < dC;
            }
            nNumInliers = 0;
            nNumInliers = 
                accumulate( vInliers.begin(), vInliers.end(), nNumInliers );
            //cout << "nNumInliers: " << nNumInliers << endl;
            dMADFactAdapt = dMADFactAdapt / ( 1. - 2*dMADFactAdapt );
        }
        // Compute final weights
        for( int ii=0; ii<nNumResiduals; ii++ ) {
            if( vInliers[ii] ) {
                mW[ii] = sq( 1. - sq(vRCentered[ii]/dC) );
            } 
            else {
                mW[ii] = 0.;
            }
        }
    }
    else { // Huber weights?
    }
    return mW;
}
