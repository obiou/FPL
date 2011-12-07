#ifndef CTRACK_IMAGE_HOLDER_H

#define CTRACK_IMAGE_HOLDER_H

#include <string.h>
#include <vector>

namespace CTrack {    
    ////////////////////////////////////////////////////////////////////////////
    /// Very basic class to simplify the function calls (it does not manage the memory)
    class ImageHolder {
    public:
        ImageHolder( unsigned char* const pImageData, 
                     const int nWidth,  const int nHeight,
                     const int nWidthStep ) :
        pImageData( pImageData ), nWidth( nWidth ), nHeight( nHeight ),
            nWidthStep( nWidthStep ) {}
        void print( const char* sMsg ) {
            std::cout << sMsg << std::endl;
            for( int ii=0; ii<nWidth; ii++ ) {
                for( int jj=0; jj<nHeight; jj++ ) {
                    std::cout << pImageData[ii+jj*nWidthStep] << " ";
                }
                std::cout << std::endl;
            }
        } 
        unsigned char* const pImageData;
        const int nWidth;
        const int nHeight;
        const int nWidthStep; 

        friend void copy( const ImageHolder* pIn, ImageHolder** pOut ) {
            unsigned char* pData = (unsigned char*)malloc( pIn->nWidthStep * pIn->nHeight );
            memcpy( pData, pIn->pImageData, pIn->nWidthStep * pIn->nHeight );
            *pOut = new ImageHolder( pData, pIn->nWidth, pIn->nHeight, pIn->nWidthStep  );
        }
    private:
        ImageHolder( const ImageHolder& );
        ImageHolder& operator=( const ImageHolder& );
    };
}

#endif
