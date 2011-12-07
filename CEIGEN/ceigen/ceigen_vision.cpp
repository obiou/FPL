#include <ceigen/ceigen_vision.h>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/SVD>

////////////////////////////////////////////////////////////////////////////////
void CEIGEN::HToSE3( const Eigen::Matrix3d& mHIn,
                     Eigen::Matrix3d& mR,
                     Eigen::Vector3d& mt,
                     bool bFix ) {
    Eigen::Matrix3d mH = mHIn;
    // Georg's fix (see appendix of thesis)
    // Fix up possibly poorly conditioned bits of the homography
    if( bFix ) {
        Eigen::JacobiSVD<Eigen::Matrix2d> svdOfTopLeftH( mH.block(0,0,2,2), Eigen::ComputeFullV );
        Eigen::Matrix2d mV = svdOfTopLeftH.matrixV();
        Eigen::Vector2d vS = svdOfTopLeftH.singularValues();
        mH /= vS(0);
        vS /= vS(0);
        double dLambda2 = vS(1);
    
        Eigen::Vector2d v2b;   // This is one hypothesis for v2b ; the other is the negative.
        v2b(0) = 0.0;
        v2b(1) = sqrt( 1.0 - (dLambda2 * dLambda2)); 
    
        Eigen::Vector2d v2aprime = v2b.transpose() * mV;    
        Eigen::Vector2d v2a = mH.row(2).segment( 0, 2 );
        double dDotProd = v2a.dot( v2aprime );
    
        if( dDotProd>0 ) { mH.row(2).segment(0,2) = v2aprime; }
        else { mH.row(2).segment(0,2) = -v2aprime; }
    }
    Eigen::Vector3d r1 = mH.col(0);
    Eigen::Vector3d r2 = mH.col(1) - ( mH.col(0).dot( mH.col(1) )/ mH.col(0).squaredNorm() ) * mH.col(0);
    r2 /= r2.norm();
    mR.col(0) = r1;
    mR.col(1) = r2;
    mR.col(2) = r1.cross( r2 );       
    mt = mH.col(2);
}

////////////////////////////////////////////////////////////////////////////////
Eigen::MatrixXd CEIGEN::make_grid( const unsigned int nWidth, 
                                   const unsigned int nHeight,
                                   const bool b3Rows,
                                   const bool bColMajor ) {
   Eigen::MatrixXd mGrid( b3Rows ? 3 : 2, nWidth*nHeight );
   if( bColMajor ) {    
        Eigen::MatrixXd m1 = Eigen::RowVectorXd::LinSpaced( nWidth, 0, nWidth-1 ).replicate( nHeight, 1 );
        mGrid.row( 0 ) = Eigen::Map<Eigen::MatrixXd>( m1.data(), 1, nWidth*nHeight );
        mGrid.row( 1 ) = Eigen::RowVectorXd::LinSpaced( nHeight, 0, nHeight-1 ).replicate( 1, nWidth );
    }
    else {
        Eigen::MatrixXd m1 = Eigen::RowVectorXd::LinSpaced( nHeight, 0, nHeight-1 ).replicate( nWidth, 1 );
        mGrid.row( 0 ) = Eigen::RowVectorXd::LinSpaced( nWidth, 0, nWidth-1 ).replicate( 1, nHeight );
        mGrid.row( 1 ) = Eigen::Map<Eigen::MatrixXd>( m1.data(), 1, nWidth*nHeight );
    }
    if( b3Rows ) {
        mGrid.row( 2 ) = Eigen::RowVectorXd::Zero( nWidth*nHeight );
    }
    return mGrid;
}

////////////////////////////////////////////////////////////////////////////////
Eigen::MatrixXf CEIGEN::make_grid_f( const unsigned int nW, 
                                     const unsigned int nH,
                                     const bool b3Rows,
                                     const bool bColMajor ) {
    return make_grid( nW, nH, b3Rows, bColMajor ).cast<float>();
}

////////////////////////////////////////////////////////////////////////////////
Eigen::MatrixXd CEIGEN::make_border( const unsigned int nWidth, 
                                     const unsigned int nHeight) {
    Eigen::MatrixXd mBorder( 2, 2*nWidth + 2*nHeight );
    mBorder.row(0).segment( 0, nWidth ) = 
        Eigen::RowVectorXd::LinSpaced( nWidth, 0, nWidth-1 );
    mBorder.row(0).segment( nWidth, nHeight ) = 
        (nWidth-1)*Eigen::MatrixXd::Ones(1,nHeight);
    mBorder.row(0).segment( nWidth+nHeight, nWidth ) = 
        Eigen::RowVectorXd::LinSpaced( nWidth, 0, nWidth-1 );
    mBorder.row(0).segment( 2*nWidth+nHeight, nHeight ) = 
        Eigen::MatrixXd::Zero(1,nHeight);

    mBorder.row(1).segment( 0, nWidth ) = 
        Eigen::MatrixXd::Zero(1,nWidth);
    mBorder.row(1).segment( nWidth, nHeight ) = 
        Eigen::RowVectorXd::LinSpaced( nHeight, 0, nHeight-1 );
    mBorder.row(1).segment( nWidth+nHeight, nWidth ) = 
        (nHeight-1)*Eigen::MatrixXd::Ones(1,nWidth);
    mBorder.row(1).segment( 2*nWidth+nHeight, nHeight ) = 
        Eigen::RowVectorXd::LinSpaced( nHeight, 0, nHeight-1 );

    return mBorder;
}
