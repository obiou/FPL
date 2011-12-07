#ifndef MISC_H
#define MISC_H

#include <sys/resource.h>
#include <sys/time.h>

#include <iostream>

////////////////////////////////////////////////////////////////////////////////
inline double Tic() {
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return  tv.tv_sec + 1e-6 * (tv.tv_usec);
}

////////////////////////////////////////////////////////////////////////////////
inline double TocMS( double t0 ) {
    return ( Tic() - t0 )*1000.;
}

////////////////////////////////////////////////////////////////////////////////
void print( const char* sMsg, unsigned char* pIm, 
            int nPatchWidth, int nPatchHeight, int nPatchWidthStep )
{
    std::cout << sMsg << std::endl;
    for( int ii=0; ii<nPatchWidth; ii++ ) {
        for( int jj=0; jj<nPatchHeight; jj++ ) {
            std::cout << pIm[ii+jj*nPatchWidthStep] << " ";
        }
        std::cout << std::endl;
    }
}

#endif
