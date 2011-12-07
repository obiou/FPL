#include <iostream>
#include <stdio.h>
#include <time.h>

#include <CTrack/TestingHelpers.h>
#include <CTrack/MatrixLogarithm.h>

using namespace std;

//////////////////////////////////////////////////////////////////////////////
///
/// Test code for the matrix logarithm
///
//////////////////////////////////////////////////////////////////////////////
int main( int, char** )
{
    //double dLie[] = {10,5,3.14/30};
    const int nSize = 8;
    //double dLie[nSize] = {.1,.2,.3,.4,.5,.6,.7,.8};
    double dLie[nSize] = {10,20,.3,.4,.5,.6,.7,1};
    std::vector<double> vLie( dLie, dLie+nSize );

    Eigen::Map<Eigen::VectorXd> vLieV( &vLie[0], vLie.size() );
    cout << "Solution:" << endl;
    cout << CTrack::MakeSumGenerators( vLieV ) << endl << endl;

    //CTrack::Homography M = CTrack::GenerateTranslation( 2, 3 );
    CTrack::Homography M( vLie );
    cout << "Matrix M:" << endl;
    cout << M.m_mH << endl << endl;
    
    cout << "logm( M ):" << endl;
    cout << CTrack::logm( M.m_mH ) << endl << endl;

    return 0;
}
