#ifndef CEIGEN_BASICS_H
#define CEIGEN_BASICS_H

#include <eigen3/Eigen/Core>

#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

namespace CEIGEN {
    ////////////////////////////////////////////////////////////////////////////////
    /// Adds a row of ones to the matrix
    template<class Derived>
        struct ProjOutput
        {
            typedef Eigen::Matrix<typename Eigen::internal::traits<Derived>::Scalar,
                Derived::RowsAtCompileTime == Eigen::Dynamic ? Eigen::Dynamic : Derived::RowsAtCompileTime+1,
                Derived::ColsAtCompileTime,
                Derived::Options
                > Type;
            typedef Eigen::Matrix<typename Eigen::internal::traits<Derived>::Scalar, 1,
                Derived::ColsAtCompileTime,
                Derived::Options
                > ColType;
        };
    template<typename Derived>
        typename ProjOutput<Derived>::Type projective( const Eigen::MatrixBase<Derived>& mM ) {
        typename ProjOutput<Derived>::Type mRet( mM.rows()+1, mM.cols() );
        mRet.block( 0, 0, mM.rows(), mM.cols() ) = mM;
        mRet.block( mRet.rows()-1, 0, 1, mM.cols() ) = ProjOutput<Derived>::ColType::Ones( 1,  mM.cols() );
        return mRet;
    }

    ////////////////////////////////////////////////////////////////////////////////
    /// Projective space to metric space by division by the last row.
    template<class Derived>
        struct MetricOutput
        {
            typedef Eigen::Matrix<typename Eigen::internal::traits<Derived>::Scalar,
                Derived::RowsAtCompileTime == Eigen::Dynamic ? Eigen::Dynamic : Derived::RowsAtCompileTime-1,
                Derived::ColsAtCompileTime//,
                //Derived::Options HAD TO REMOVE THIS...
                > Type;
        };
    template<typename Derived>
        typename MetricOutput<Derived>::Type metric( const Eigen::MatrixBase<Derived>& mM ) {
        typename MetricOutput<Derived>::Type mRet( mM.rows()-1, mM.cols() );
        for( int nRow=0; nRow<mM.rows()-1; nRow++ ) {
            for( int nCol=0; nCol<mM.cols(); nCol++ ) {
                mRet( nRow, nCol ) = mM( nRow, nCol )/mM( mM.rows()-1, nCol );
            }
        }
        return mRet;
    }

    ////////////////////////////////////////////////////////////////////////////////
    /// Not too sure how to deal with this case.
    /// This version is inefficient.
    template<typename Lhs, typename Rhs, int Mode>
        struct MetricOutputGP
        {
            typedef Eigen::Matrix<typename Eigen::internal::traits<Lhs>::Scalar, 
                Lhs::RowsAtCompileTime == Eigen::Dynamic ? Eigen::Dynamic : Lhs::RowsAtCompileTime-1, 
                Rhs::ColsAtCompileTime,
                Lhs::Base::Options > Type;
            typedef Eigen::Matrix<typename Eigen::internal::traits<Lhs>::Scalar, 
                Lhs::RowsAtCompileTime, 
                Rhs::ColsAtCompileTime,
                Lhs::Base::Options > InnerType;
        };

    template<typename Lhs, typename Rhs, int Mode>
        typename MetricOutputGP<Lhs,Rhs,Mode>::Type 
        metric( const Eigen::MatrixBase<Eigen::GeneralProduct<Lhs, Rhs,Mode> >& mM ) {
        typename MetricOutputGP<Lhs,Rhs,Mode>::InnerType mMConv = mM;
        return metric( mMConv );
    }

    ////////////////////////////////////////////////////////////////////////////////
    /// Divides all values of a column by the last value of the column
    /// and removes the last row.
    /// Spiffy matlab-like one-liner, but happens to be slower than iterating.
    template<typename Derived>
        inline Derived metric_slow( const Eigen::MatrixBase<Derived>& mM ) {
        return mM.block( 0, 0, mM.rows()-1, mM.cols() ).array() / mM.row( 2 ).replicate( 2, 1 ).array();
    }

    ////////////////////////////////////////////////////////////////////////////
    template <class Derived>
        std::fstream& operator>>( std::fstream& fs, Eigen::MatrixBase<Derived>& mM ) {
        typedef typename Eigen::internal::traits<Derived>::Scalar LScalar;
        std::string sLine;
        int nHeight=0; int nWidth;
        std::vector<LScalar> vVals;
        LScalar dVal;
        while( getline(fs, sLine) ) {
            std::stringstream ss( std::stringstream::in | std::stringstream::out );
            ss << sLine;
            while( ss >> dVal ) {
                vVals.push_back( dVal );
            }
            if(nHeight==0) nWidth = vVals.size();
            nHeight++;
        }
        if( int(vVals.size()) != nWidth*nHeight ) {
            std::cerr << "ERROR: while reading matrix from file, missing data." << std::endl;
            return fs;
        }
        mM = Derived( nHeight, nWidth );
        for( int nRow=0; nRow<nHeight; nRow++ ) {
            for( int nCol=0; nCol<nWidth; nCol++ ) {
                mM( nRow, nCol ) = vVals[ nRow*nWidth + nCol ];
            }
        }
        return fs;
    }
}

#endif

