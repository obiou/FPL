#include <COpenCV/copencv_basics.h>

////////////////////////////////////////////////////////////////////////////////
IplImage* COPENCV::ExtractPatch( IplImage* pImage,
                                 const int nBegX,
                                 const int nBegY,
                                 const int nPatchWidth,
                                 const int nPatchHeight,
                                 IplImage* pPatchIn
                                 ) {
    IplImage* pPatch = pPatchIn == NULL ? 
        cvCreateImage( cvSize( nPatchWidth, nPatchHeight ),
                       pImage->depth,
                       pImage->nChannels ) :
        pPatchIn;
    CvRect rect = cvRect( nBegX, nBegY, nPatchWidth, nPatchHeight );
    cvSetImageROI( pImage, rect ); // Sets the region of interest
    cvCopy( pImage, pPatch, NULL );
    cvResetImageROI( pImage );
    return pPatch;
}

////////////////////////////////////////////////////////////////////////////
template<class T>
IplImage* COPENCV::ExtractPatch( IplImage* pImage,
                                 const std::vector<std::pair<T,T> > vBox,
                                 IplImage* pPatchIn
                                 ) {
    return ExtractPatch( pImage, 
                         vBox[0].first, vBox[0].second,
                         vBox[2].first  - vBox[0].first + 1, 
                         vBox[2].second - vBox[0].second + 1,
                         pPatchIn );
}

////////////////////////////////////////////////////////////////////////////
double COPENCV::RMS( const IplImage* pI1,
                     const IplImage* pI2
                     ) {
    double dSum = 0;
    assert( pI1->width     == pI2->width );
    assert( pI1->height    == pI2->height );
    assert( pI1->widthStep == pI2->widthStep );
    unsigned char* d1 = (unsigned char*) pI1->imageData;
    unsigned char* d2 = (unsigned char*) pI2->imageData;
    const int nPadding = pI1->widthStep - pI1->width;
    for( int nRow=0; nRow<pI1->height; nRow++ ) {
        for( int nCol=0; nCol<pI1->width; nCol++ ) {
            dSum += (*d1++ - *d2++ );
        }
        d1 += nPadding;
        d2 += nPadding;
    }
    return dSum;
}

////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////
#define INST( T ) \
    template                                            \
    IplImage* COPENCV::ExtractPatch                                     \
    ( IplImage*,                                                        \
      const std::vector<std::pair<T,T> >,                               \
      IplImage*                                                         \
      );

INST( int )
INST( float )
INST( double )
