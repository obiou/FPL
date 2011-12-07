#include <ceigen/ceigen_basics.h>
#include <ceigen/ceigen_image.h>
#include <ceigen/ceigen_vision.h>

////////////////////////////////////////////////////////////////////////////////
namespace CEIGEN {
    class NoMask { //: public MaskFunctor { // do not inherit explicitly: too expensive (virtual function)
    public: 
        NoMask() {}
        NoMask( int, int ) {}
        ~NoMask() {}
        void fwd() {}
        void set( bool ) {}
        void end() {}
        void set_to_nan( double*, int, int ) {}
    };

    ////////////////////////////////////////////////////////////////////////////////
    template< class MaskFunctorT >
    inline void BilinearInterpolation( const int nWM, const int nHM, 
                                       const int nImageWidthStep,
                                       const unsigned char* pImage,
                                       const double fXc, const double fYc,
                                       unsigned char* pWarpedPatch,
                                       MaskFunctorT& maskFctr
                                       )
    {
        // Truncates the values
        const int nFloorXc = (int)fXc;
        const int nFloorYc = (int)fYc;

        if( nFloorXc >= 0 &&
            nFloorYc >= 0 &&
            nFloorXc < nWM && 
            nFloorYc < nHM ) {

            // Main case: points inside the image
            int nCoord = nFloorXc + nFloorYc*nImageWidthStep;

            double fV00 = pImage[ nCoord ];
            double fV10 = pImage[ nCoord + 1  ];
            nCoord += nImageWidthStep;
            double fV01 = pImage[ nCoord ];
            double fV11 = pImage[ nCoord + 1 ];

#if 0
            // This is slower with fewer mult ! :
            const double fTmp1 = fV00 + (fXc-nFloorXc)*(fV10-fV00);
            const double fTmp2 = fV01 + (fXc-nFloorXc)*(fV11-fV01);
            //*pWarpedPatch = (char) fTmp1 + (fYc-nFloorYc)*( fTmp2 - fTmp1 );
            *pWarpedPatch = (char) fTmp1 + (fYc-nFloorYc)*fTmp2 - (fYc-nFloorYc)*fTmp1;
#else
            *pWarpedPatch = (char) ( fV00 +
                                     (fXc-nFloorXc)*(fV10-fV00)+
                                     (fYc-nFloorYc)*(fV01-fV00)-
                                     (fXc-nFloorXc)*(fYc-nFloorYc)*(fV01-fV00+fV10-fV11) );
#endif

            //*pWarpedPatchMask = 1;
            maskFctr.set( true );
        } 
        else { 
            //printf( "WARNING: out of image in Warp\n" );
            *pWarpedPatch     = 0;
            //*pWarpedPatchMask = 0;
            maskFctr.set( false );
        }
    }
}

////////////////////////////////////////////////////////////////////////////
Eigen::MatrixXd CEIGEN::make_warp_grid( const int nPatchWidth,
                                        const int nPatchHeight ) {
    return CEIGEN::make_grid( nPatchWidth, nPatchHeight,
                              false, false );
}

////////////////////////////////////////////////////////////////////////////////
void CEIGEN::Warp( const unsigned char* pImageIn,
                   const int nImageWidth,
                   const int nImageHeight,
                   const int nImageWidthStep,
                   const Eigen::Matrix3d& H,
                   unsigned char* pImageOut,
                   const int nPatchWidth,
                   const int nPatchHeight,
                   const int nPatchWidthStep,
                   Eigen::MatrixXd grid_tmp,
                   Eigen::MatrixXd warped_grid_tmp
                   ) {
    warped_grid_tmp = CEIGEN::metric( H*CEIGEN::projective( grid_tmp ) );
    NoMask noMask;
    const int nPadding = nPatchWidthStep - nPatchWidth;

    double* pData = warped_grid_tmp.data();
    for( int nRow=0; nRow<nPatchHeight; nRow++ ) {
        for( int nCol=0; nCol<nPatchWidth; nCol++ ) { 
            const double f1 = *pData++;
            const double f2 = *pData++;
            BilinearInterpolation( nImageWidth-1, nImageHeight-1, 
                                   nImageWidthStep, pImageIn,
                                   f1,//warped_grid_tmp( 0, nCol + nRow*nPatchWidth ),
                                   f2,//warped_grid_tmp( 1, nCol + nRow*nPatchWidth ),
                                   pImageOut, noMask );
            pImageOut++;
        }
        pImageOut += nPadding;
    }     
}

////////////////////////////////////////////////////////////////////////////////
void CEIGEN::Warp( const unsigned char* pImageIn,
                   const int nImageWidth,
                   const int nImageHeight,
                   const int nImageWidthStep,
                   const Eigen::Matrix3d& H,
                   unsigned char* pImageOut,
                   const int nPatchWidth,
                   const int nPatchHeight,
                   const int nPatchWidthStep
                   ) {
    Eigen::MatrixXd grid_tmp = CEIGEN::make_grid( nPatchWidth, nPatchHeight,
                                                  false, false );
    Eigen::MatrixXd warped_grid_tmp = Eigen::MatrixXd::Zero( 2, nPatchWidth*nPatchHeight );
    CEIGEN::Warp( pImageIn, nImageWidth, nImageHeight, nImageWidthStep, H,
                  pImageOut, nPatchWidth, nPatchHeight, nPatchWidthStep,
                  grid_tmp, warped_grid_tmp );
}

#if CEIGEN_HAS_OPENCV
////////////////////////////////////////////////////////////////////////////////
void CEIGEN::Warp( IplImage* pImageIn,
                   const Eigen::Matrix3d& H,
                   IplImage* pImageOut ) {
    Warp( (const unsigned char*)pImageIn->imageData, pImageIn->width, pImageIn->height, pImageIn->widthStep,
          H,
          (unsigned char*)pImageOut->imageData, pImageOut->width, pImageOut->height, pImageOut->widthStep );
}

////////////////////////////////////////////////////////////////////////////////
void CEIGEN::Warp( IplImage* pImageIn,
                   const Eigen::Matrix3d& H,
                   IplImage* pImageOut,
                   Eigen::MatrixXd grid_tmp,
                   Eigen::MatrixXd warped_grid_tmp ) {
    Warp( (const unsigned char*)pImageIn->imageData, pImageIn->width, pImageIn->height, pImageIn->widthStep,
          H,
          (unsigned char*)pImageOut->imageData, pImageOut->width, pImageOut->height, pImageOut->widthStep,
          grid_tmp, warped_grid_tmp );
}
#endif
