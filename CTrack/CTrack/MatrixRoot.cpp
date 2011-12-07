#include <CTrack/MatrixRoot.h>

#include <assert.h>
#include <cmath>
#include <iostream>

//#include <eigen3/Eigen/Cholesky>
#include <eigen3/Eigen/LU>

////////////////////////////////////////////////////////////////////////////////
/**
 * -- Matlab code --
 * I = eye(size(H,2));
 * scale = norm(H,'inf');
 * Xk = H/scale;
 * Zk = 1/scale*I;
 *
 * for i=1:30
 *  V = (3*I+Zk*Xk)*inv(I+3*Zk*Xk);
 *  Xk = Xk*V;
 *  Zk = V*Zk;
 * end
 **/
bool CTrack::sqrtm(
                   const Eigen::MatrixXd& M, 
                   Eigen::MatrixXd& sqrtM, 
                   double dMaxErr
                   )
{
    typedef double T;
    int nN = M.rows();
    assert( nN == M.rows() && nN == M.cols() );
    assert( nN == sqrtM.rows() && nN == sqrtM.cols() );

    T norm_Res;

    int iter = 0;
    Eigen::MatrixXd Zk( nN, nN );
    Eigen::MatrixXd V( nN, nN );
    Eigen::MatrixXd W( nN, nN );
    Eigen::MatrixXd Res( nN, nN );
    Eigen::MatrixXd I( nN, nN );
    Eigen::MatrixXd I3( nN, nN );
    I.setIdentity();
    I3.setIdentity();
    I3 *= 3.;
    Zk.setIdentity();

    // Initialise values
    T scale = M.lpNorm<Eigen::Infinity>();
    sqrtM = M/scale;
    Zk /= scale;
    Res = sqrtM*sqrtM-M;
    norm_Res = Res.lpNorm<Eigen::Infinity>();

    while((iter++<MAX_ITER_ROOT)&&(norm_Res>dMaxErr)) {
        //M = (I*3+Zk*sqrtM)*(I+(Zk*sqrtM)*3).inverse();
        W = Zk*sqrtM;
        V = (I3+W)*(I+W*3).inverse();
        sqrtM *= V;
        Zk = V*Zk;
  
        Res = sqrtM*sqrtM-M;
        norm_Res = Res.lpNorm<Eigen::Infinity>();
    }

    if(norm_Res>dMaxErr) {
        std::cout << "Error: " << norm_Res << std::endl;
        std::cout << "Iter: " << iter << std::endl;
    } 

    if(norm_Res>dMaxErr)
        return false;

    return true;
}

////////////////////////////////////////////////////////////////////////////////
Eigen::MatrixXd CTrack::sqrtm(
                              const Eigen::MatrixXd& M 
                              )
{
    bool bRetVal = true;
    Eigen::MatrixXd sqrtM( M.rows(), M.cols() );

#ifndef NDEBUG
    bRetVal = 
#endif
        CTrack::sqrtm( M, sqrtM, 1e-10 );
#ifndef NDEBUG
    assert( bRetVal );
#endif
    return sqrtM;
}

////////////////////////////////////////////////////////////////////////////////
/**
 * -- Matlab code --
 * scale = norm(H,'fro');
 * Xk = 1/scale*eye(size(H,2));
 * Yk = 1/scale^lambda*H;
 *
 * iter_step = @(A) ( 1/lambda*( (lambda+1)*eye(3)-A ));
 *
 * for i=1:30
 *  Xk = Xk*iter_step(Yk);
 *  Yk = iter_step(Yk)^lambda*Yk;
 * end
 * Xk = H*Xk^(lambda-1);
 */
bool CTrack::rootm( 
                   const Eigen::MatrixXd& M,
                   Eigen::MatrixXd& rootM,
                   double dMaxErr,
                   int nP
                    )
{
    typedef double T;
    assert( nP > 0 );

    if( nP == 1 ) {
        rootM = M;
        return true;
    }

    double dLog2p = 
#if !defined(_WIN32) && !defined(_WIN64)
        // no such thing in windows...
        log2( nP );
#else
    log( nP )/log((T)2);
#endif
    
    if(ceil(dLog2p)!=dLog2p) {
        std::cout << "Matrix square currently only works with " <<
            "powers of 2 because it's simpler to calculate the power of a matrix..." << std::endl;
        assert(ceil(dLog2p)==dLog2p);
    }

    int nN = M.rows();
    assert( nN == M.rows() && nN == M.cols() );
    assert( nN == rootM.rows() && nN == rootM.cols() );

    bool bEven = fmod( nP, 2 )==0;

    Eigen::MatrixXd Yk, V, Xk;
    Eigen::MatrixXd sqrtM = Eigen::MatrixXd::Identity( nN, nN );
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity( nN, nN );

    // Initialise values
    T invscale;
    T invp = (T)(1./nP);
    /*
     * If even start by calculating the
     * square root for better stability
     */
    if( bEven ) {
        if( !CTrack::sqrtm( M, sqrtM, dMaxErr ) )
            return false;

        if( nP == 2 ) {
            rootM = sqrtM;
            return true;
        }      

        nP /= 2;
        invp *= 2;
        dLog2p--;

        //invscale = 1/Yk.operatorInfNorm();
        invscale = 1/sqrtM.norm();

        Yk = sqrtM*pow( invscale, nP ); 
        rootM = I*invscale;
        Xk = rootM;
    } else {
        invscale = 1/M.lpNorm<Eigen::Infinity>();
        Yk = M*pow( invscale, nP ); 
        rootM = I*invscale;
        Xk = rootM;
    }
  
    double dIncrVal = 1;
    int nIter = 0;

    while((nIter++<MAX_ITER_ROOT)&&(dIncrVal>1e-14)) {
        V = ( (nP+1)*I-Yk )*invp;
        Xk = Xk*V;

        // Calculate V^2^log2(p);
        // This should be changed to enable
        // roots that are not powers of 2
        for( int ii=0; ii<dLog2p; ii++ )
            V *= V;

        Yk = V*Yk;
  
        // Only stop iterations when the value does
        // not change. This is not a good stopping 
        // policy as we could be far from the correct
        // value and not detect it... FIX ME - HOW?
        // (calculating the product is expensive)
        dIncrVal = (Xk-rootM).lpNorm<Eigen::Infinity>();
        rootM = Xk;
    }

    if( bEven ) {
        // Calculate rootM^(p-1)= rootM^p*rootM^{-1}
        V = rootM;

        // Calculate V^2^log2(p);
        // This should be changed to enable
        // roots that are not powers of 2
        for( int ii=0; ii<dLog2p; ii++ )
            V *= V;

        V *= rootM.inverse();

        rootM = sqrtM*V;
    }

    // Expensive final test...
    V = rootM;
    if( bEven ) {
        for( int ii=0; ii<dLog2p+1; ii++ )
            V *= V;
    } else {
        for( int ii=0; ii<dLog2p; ii++ )
            V *= V;
    }

    dIncrVal = (M-V).lpNorm<Eigen::Infinity>();

    if( dIncrVal>dMaxErr ) {
        std::cout << "Error: " << dIncrVal << std::endl;
        std::cout << "Iter: " << nIter << std::endl;
    }

    return dIncrVal < dMaxErr;
}

////////////////////////////////////////////////////////////////////////////////
Eigen::MatrixXd CTrack::rootm( 
                              const Eigen::MatrixXd& M,
                              int nP
                               )
{
    bool bRetVal = true;
    Eigen::MatrixXd rootM( M.rows(), M.cols() );

#ifndef NDEBUG
    bRetVal = 
#endif
        CTrack::rootm( M, rootM, 1e-10, nP );

    assert( bRetVal );
    return rootM;
}
