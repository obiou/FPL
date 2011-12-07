#ifndef OPENCV_HELPERS_H
#define OPENCV_HELPERS_H

#include <cv.h>
#include <highgui.h>

#include <iostream>
#include <sstream>

#include <COpenCV/copencv_basics.h>

#ifdef COPENCV_HAS_CVARS
#  include <COpenCV/OpenCVConsole.h>
#endif

namespace COPENCV {

    ////////////////////////////////////////////////////////////////////////////////
    template<class T>
    inline void plot( IplImage* pImage,
               const std::pair<T,T>& p,
               const int r, const int g, const int b, const int s ) 
    {
        CvPoint point = cvPoint( cvRound( p.first ), cvRound( p.second ) );
        cvCircle( pImage, point, s, CV_RGB(r,g,b) );
    }

    ////////////////////////////////////////////////////////////////////////////////
    template<class T>
    inline void plot( IplImage* pImage,
               const std::pair<T,T>& p,
               const int nOffsetX, const int nOffsetY,
               const int r, const int g, const int b, const int s ) 
    {
        CvPoint point = cvPoint( cvRound( p.first )  + nOffsetX, 
                                 cvRound( p.second ) + nOffsetY );
        cvCircle( pImage, point, s, CV_RGB(r,g,b) );
    }

    ////////////////////////////////////////////////////////////////////////////////
    template<class T>
    inline void plot( IplImage* pImage,
               const std::vector<std::pair<T,T> >& vP,
               const int nOffsetX, const int nOffsetY,
               const int r, const int g, const int b, const int s ) 
    {
        for( size_t ii=0; ii < vP.size(); ++ii ) {
            plot<T>( pImage, vP[ii], nOffsetX, nOffsetY, r, g ,b , s );
        }
    }

    ////////////////////////////////////////////////////////////////////////////////
    template<class T>
    inline void plot( IplImage* pImage,
               const std::vector<std::pair<T,T> >& vP,
               const int r, const int g, const int b, const int s ) 
    {
        plot<T>( pImage, vP, 0, 0, r, g ,b , s );
    }

    ////////////////////////////////////////////////////////////////////////////////
    template<class T,class T2>
    inline void plot( IplImage* pImage,
               const std::pair<T,T>& p1,
               const std::pair<T2,T2>& p2,
               const int nOffsetX,
               const int nOffsetY,
               const int r, const int g, const int b )
    {
        CvPoint point1 = cvPoint( cvRound( p1.first ), cvRound( p1.second ) );
        CvPoint point2 = cvPoint( cvRound( p2.first )  + nOffsetX, 
                                  cvRound( p2.second ) + nOffsetY );
        cvLine( pImage, point1, point2, CV_RGB(r,g,b) );
    }

    ////////////////////////////////////////////////////////////////////////////////
    inline void StereoSize( const IplImage* pImage1,
                     const IplImage* pImage2,
                     const bool bHorizontal,
                     int& nStereoImageWidth, 
                     int& nStereoImageHeight
                     ) {
        if( bHorizontal ) {
            nStereoImageWidth  = pImage1->width + pImage2->width;
            nStereoImageHeight = std::max( pImage1->height, pImage2->height );
        }
        else {
            nStereoImageWidth  = std::max( pImage1->width, pImage2->width );
            nStereoImageHeight = pImage1->height+pImage2->height;
        }
    }

    ////////////////////////////////////////////////////////////////////////////////
    inline void MakeStereo( const IplImage* pImage1,
                     const IplImage* pImage2,
                     IplImage** ppGlobalImage,
                     bool bHorizontal = true,
                     bool bColour = false
                     ) 
    {
        IplImage*& pGlobalImage = *ppGlobalImage;
        int nGlobalImageWidth, nGlobalImageHeight;
        StereoSize( pImage1, pImage2, bHorizontal, 
                    nGlobalImageWidth, nGlobalImageHeight );

        if( pGlobalImage != NULL ) {
            // Check for correct size
            if( pGlobalImage->width  != nGlobalImageWidth ||
                pGlobalImage->height != nGlobalImageHeight ) {
                cvReleaseImage( &pGlobalImage );
                pGlobalImage = NULL;
            }
        }

        if( pGlobalImage == NULL ) {
            pGlobalImage = cvCreateImage( cvSize( nGlobalImageWidth, nGlobalImageHeight ),
                                          IPL_DEPTH_8U, bColour ? 3 : 1 );
        }
        unsigned char* pImageData = NULL;

        // Copy first image
        pImageData = (unsigned char*)pGlobalImage->imageData;
        if( pImage1->nChannels == 1 ) {
            if( !bColour ) {
                for( int ii=0; ii < pImage1->height; ++ii, pImageData += pGlobalImage->widthStep ) {
                    memcpy( pImageData, pImage1->imageData + ii * pImage1->widthStep, pImage1->width );
                }
            }
            else {
                for( int ii=0; ii < pImage1->height; ++ii, pImageData += pGlobalImage->widthStep ) {
                    unsigned char* pImage1Row = (unsigned char*) pImage1->imageData + ii * pImage1->widthStep;
                    for( int jj=0; jj<pImage1->width; jj++ ) {
                        pImageData[3*jj] = pImage1Row[jj];
                        pImageData[3*jj+1] = pImage1Row[jj];
                        pImageData[3*jj+2] = pImage1Row[jj];
                    }
                }
            }
        }
        else if( pImage1->nChannels == 3 ) {
            if( !bColour ) {
                for( int ii=0; ii < pImage1->height; ++ii, pImageData += pGlobalImage->widthStep ) {
                    unsigned char* pImage1Row = (unsigned char*)pImage1->imageData + ii * pImage1->widthStep;
                    for( int jj=0; jj<pImage1->width; jj++ ) {
                        pImageData[jj] = ( pImage1Row[3*jj]   +
                                           pImage1Row[3*jj+1] +
                                           pImage1Row[3*jj+2] )/3;
                    }
                }
            }
            else {
                for( int ii=0; ii < pImage1->height; ++ii, pImageData += pGlobalImage->widthStep ) {
                    memcpy( pImageData, pImage1->imageData + ii * pImage1->widthStep, pImage1->widthStep );
                }
            }
        }
        else {
            std::cerr << "ERROR: Number of channels not supported." << std::endl;
            return;
        }
                 

        // Copy second image
        if( bHorizontal ) {
            if( !bColour ) {
                pImageData = (unsigned char*)pGlobalImage->imageData + pImage1->width;
            }
            else {
                pImageData = (unsigned char*)pGlobalImage->imageData + 3*pImage1->width;
            }
        }
        else {
            pImageData = (unsigned char*)pGlobalImage->imageData + pImage1->height*pGlobalImage->widthStep;
        }
        if( pImage2->nChannels == 1 ) {
            if( !bColour ) {
                for( int ii=0; ii < pImage2->height; ++ii, pImageData += pGlobalImage->widthStep ) {
                    memcpy( pImageData, pImage2->imageData + ii * pImage2->widthStep, pImage2->width );
                }
            }
            else {
                for( int ii=0; ii < pImage2->height; ++ii, pImageData += pGlobalImage->widthStep ) {
                    unsigned char* pImage2Row = (unsigned char*)pImage2->imageData + ii * pImage2->widthStep;
                    for( int jj=0; jj<pImage2->width; jj++ ) {
                        pImageData[3*jj]   = pImage2Row[jj];
                        pImageData[3*jj+1] = pImage2Row[jj];
                        pImageData[3*jj+2] = pImage2Row[jj];
                    }
                }
            }
        }
        else if( pImage2->nChannels == 3 ) {
            if( !bColour ) {
                for( int ii=0; ii < pImage2->height; ++ii, pImageData += pGlobalImage->widthStep ) {
                    unsigned char* pImage2Row = (unsigned char*)pImage2->imageData + ii * pImage2->widthStep;
                    for( int jj=0; jj<pImage2->width; jj++ ) {
                        pImageData[jj] = ( pImage2Row[3*jj] +
                                           pImage2Row[3*jj+1] +
                                           pImage2Row[3*jj+2] )/3;
                    }
                }
            }
            else {
                for( int ii=0; ii < pImage2->height; ++ii, pImageData += pGlobalImage->widthStep ) {
                    memcpy( pImageData, pImage2->imageData + ii * pImage2->widthStep, pImage2->widthStep );
                }
            }
        }
        else {
            std::cerr << "ERROR: Number of channels not supported." << std::endl;
            return;
        }
    }

    ////////////////////////////////////////////////////////////////////////////////
    inline bool get_colour( const char cColour, int& r, int& g, int& b ) {
        switch( cColour ) {
        case 'b':
            r = 0; g = 0; b = 255;
            break;
        case 'g':
            r = 0; g = 255; b = 0;
            break;
        case 'r':
            r = 255; g = 0; b = 0;
            break;
        case 'c':
            r = 0; g = 255; b = 255;
            break;
        case 'm':
            r = 255; g = 0; b = 255;
            break;
        case 'y':
            r = 255; g = 255; b = 0;
            break;
        case 'k':
            r = 0; g = 0; b = 0;
            break;
        default:
            return false;
        }
        return true;
    }

    ////////////////////////////////////////////////////////////////////////////////
    inline bool known_point_type( const char cChar ) {
        return cChar == 'n' || cChar == '.' || cChar == 'o' || 
            cChar == 'x' ||
            cChar == '+' || cChar == '*' || cChar == 's' ||
            cChar == 'd';
    }

    ////////////////////////////////////////////////////////////////////////////////
    inline bool parse_plot_options( std::string sOptions,
                             int& r, int& g, int& b, 
                             char& cPointType, 
                             char& cLineType, int& nSize ) {
        r = 0; g = 0; b = 0; cPointType = 'n'; cLineType = '-'; nSize = 6;   
        if( sOptions == "" ) {
            return true;
        }
        else {
            int nIndex = 0;
            if( get_colour( sOptions[nIndex], r, g, b ) ) {
                nIndex++;
            }
            if( (int) sOptions.size() <= nIndex ) return true;
            if( known_point_type( sOptions[nIndex] ) ) {
                cPointType = sOptions[nIndex];
                cLineType = 'n';
                nIndex++;
            }
            if( (int) sOptions.size() <= nIndex ) return true;
            if( sOptions[nIndex] == '-' ) {
                cLineType = '-';
                nIndex++;
            }
            if( (int) sOptions.size() <= nIndex ) return true;
            std::istringstream iss( sOptions.substr( nIndex ) );
            if( !( iss >> std::dec >> nSize ).fail() ) {
                return true;
            }
        }

        return false;
    }

    ////////////////////////////////////////////////////////////////////////////////
    template<class T>
        inline void pair_reader( const std::pair<T,T>& nP, int& nX, int& nY ) {
        nX = nP.first; nY = nP.second;
    }

    ////////////////////////////////////////////////////////////////////////////////
    inline void draw_plus( IplImage* pImage, const int nX, const int nY, 
                    const int r, const int g, const int b, const int s ) {
        const int nAdd = s/2;
        cvLine( pImage, 
                cvPoint( nX, nY ), cvPoint( nX+nAdd, nY ),
                CV_RGB(r,g,b) );
        cvLine( pImage, 
                cvPoint( nX, nY ), cvPoint( nX-nAdd, nY ),
                CV_RGB(r,g,b) );
        cvLine( pImage, 
                cvPoint( nX, nY ), cvPoint( nX, nY+nAdd ),
                CV_RGB(r,g,b) );
        cvLine( pImage, 
                cvPoint( nX, nY ), cvPoint( nX, nY-nAdd ),
                CV_RGB(r,g,b) );
    }

    ////////////////////////////////////////////////////////////////////////////////
    inline void draw_cross( IplImage* pImage, const int nX, const int nY, 
                     const int r, const int g, const int b, const int s ) {
        
        const int nAdd = s/2;
        cvLine( pImage, 
                cvPoint( nX, nY ), cvPoint( nX+nAdd, nY+nAdd ),
                CV_RGB(r,g,b) );
        cvLine( pImage, 
                cvPoint( nX, nY ), cvPoint( nX-nAdd, nY-nAdd ),
                CV_RGB(r,g,b) );
        cvLine( pImage, 
                cvPoint( nX, nY ), cvPoint( nX-nAdd, nY+nAdd ),
                CV_RGB(r,g,b) );
        cvLine( pImage, 
                cvPoint( nX, nY ), cvPoint( nX+nAdd, nY-nAdd ),
                CV_RGB(r,g,b) );
    }

    ////////////////////////////////////////////////////////////////////////////////
    inline void draw_square( IplImage* pImage, const int nX, const int nY, 
                      const int r, const int g, const int b, const int s ) {
        const int nAdd = s/2;
        cvLine( pImage, 
                cvPoint( nX-nAdd, nY-nAdd ), cvPoint( nX+nAdd, nY-nAdd ),
                CV_RGB(r,g,b) );
        cvLine( pImage, 
                cvPoint( nX+nAdd, nY-nAdd ), cvPoint( nX+nAdd, nY+nAdd ),
                CV_RGB(r,g,b) );
        cvLine( pImage, 
                cvPoint( nX+nAdd, nY+nAdd ), cvPoint( nX-nAdd, nY+nAdd ),
                CV_RGB(r,g,b) );
        cvLine( pImage, 
                cvPoint( nX-nAdd, nY+nAdd ), cvPoint( nX-nAdd, nY-nAdd ),
                CV_RGB(r,g,b) );
    }

    ////////////////////////////////////////////////////////////////////////////////
    inline void draw_diam( IplImage* pImage, const int nX, const int nY, 
                    const int r, const int g, const int b, const int s ) {
        const int nAdd = s/2;
        cvLine( pImage, 
                cvPoint( nX, nY-nAdd ), cvPoint( nX+nAdd, nY ),
                CV_RGB(r,g,b) );
        cvLine( pImage, 
                cvPoint( nX+nAdd, nY ), cvPoint( nX, nY+nAdd ),
                CV_RGB(r,g,b) );
        cvLine( pImage, 
                cvPoint( nX, nY+nAdd ), cvPoint( nX-nAdd, nY ),
                CV_RGB(r,g,b) );
        cvLine( pImage, 
                cvPoint( nX-nAdd, nY ), cvPoint( nX, nY-nAdd ),
                CV_RGB(r,g,b) );
    }

    ////////////////////////////////////////////////////////////////////////////////
    inline void draw_point( IplImage* pImage, const int nX, const int nY, 
                     const char cPointType,
                     const int r, const int g, const int b, const int nSize ) {
        switch( cPointType ) {
        case 'n' :
            break;
        case '.':
            cvCircle( pImage, cvPoint( nX, nY ), 1, CV_RGB(r,g,b) );
            break;
        case 'o':
            cvCircle( pImage, cvPoint( nX, nY ), nSize, CV_RGB(r,g,b) );
            break;
        case 'x': 
            draw_cross( pImage, nX, nY, r, g, b, nSize );
            break;
        case '+': 
            draw_plus( pImage, nX, nY, r, g, b, nSize );
            break;
        case '*': 
            draw_plus( pImage, nX, nY, r, g, b, nSize );
            draw_cross( pImage, nX, nY, r, g, b, 0.71*nSize );
            break;
        case 's': 
            draw_square( pImage, nX, nY, r, g, b, nSize );
            break;
        case 'd': 
            draw_diam( pImage, nX, nY, r, g, b, nSize );
            break;                
        default:
            std::cerr << "ERROR: in plot unkown line type" << std::endl;
        }
    }

    ////////////////////////////////////////////////////////////////////////////////
    template<class InputIterator, class Functor >
        inline void plot( IplImage* pImage,
                   InputIterator first, InputIterator last, Functor& reader, 
                   const std::string& sOptions,
                   const int nWidthOffset = 0,
                   const int nHeightOffset = 0
                   ) {
        int r = 0; int g = 0; int b = 0;
        char cPointType; char cLineType; int nSize;
        if( !parse_plot_options( sOptions, r, g, b, cPointType, cLineType, nSize ) ) {
            std::cout << "ERROR: problem parsing plot options." << std::endl;
            parse_plot_options( "", r, g, b, cPointType, cLineType, nSize );
        }

        bool bJoined = cLineType != 'n';

        int nPrevX = 0, nPrevY = 0;
        int nX, nY;
        int nThickness = 1;
        int nLineType = 8;
        int nShift = 0;
        bool bFirst = true;
            
        for( ; first!=last; ++first ) {
            reader( *first, nX, nY );
            nX += nWidthOffset; nY += nHeightOffset;
            draw_point( pImage, nX, nY, 
                        cPointType, r, g, b , nSize );
            
            if( bJoined && !bFirst ) {
                cvLine( pImage, 
                        cvPoint( nPrevX, nPrevY ), cvPoint( nX, nY ),
                        CV_RGB(r,g,b), nThickness, nLineType, nShift );
            }
            bFirst = false;
            nPrevX = nX;
            nPrevY = nY;
        }
    }

    ////////////////////////////////////////////////////////////////////////////////
    /// Stereo plotting
    template<class InputIterator1, class InputIterator2,
        class Functor1, class Functor2 >
        inline void plot2( IplImage* pImage,
                    InputIterator1 first1, InputIterator1 last1, Functor1& reader1, 
                    InputIterator2 first2, InputIterator2 last2, Functor2& reader2, 
                    const std::string& sOptions,
                    const int nWidthOffset,
                    const int nHeightOffset
                    ) {
        int r = 0; int g = 0; int b = 0;
        char cPointType; char cLineType; int nSize;
        if( !parse_plot_options( sOptions, r, g, b, cPointType, cLineType, nSize ) ) {
            std::cout << "ERROR: problem parsing plot options." << std::endl;
            parse_plot_options( "", r, g, b, cPointType, cLineType, nSize );
        }

        bool bJoined = cLineType != 'n';

        int nX1, nY1, nX2, nY2;
        int nThickness = 1;
        int nLineType = 8;
        int nShift = 0;
            
        for( ; first1 != last1 && first2 != last2; ++first1, ++first2 ) {
            reader1( *first1, nX1, nY1 );
            reader2( *first2, nX2, nY2 );
            nX2 += nWidthOffset;
            nY2 += nHeightOffset;
            draw_point( pImage, nX1, nY1, 
                        cPointType, r, g, b, nSize );
            draw_point( pImage, nX2, nY2, 
                        cPointType, r, g, b, nSize );

            if( bJoined ) {
                cvLine( pImage, 
                        cvPoint( nX1, nY1 ), cvPoint( nX2, nY2 ),
                        CV_RGB(r,g,b), nThickness, nLineType, nShift );
            }
        }
    }

    ////////////////////////////////////////////////////////////////////////////////
    class Figure {
    public:
#if 0
        ////////////////////////////////////////////////////
    Figure( const bool bColour = true ) 
        : m_sFigureName( "Figure" ), 
            m_bAllocated( false ),
            m_pDrawingBuffer( NULL ),
            m_bColour( bColour ) ,
            m_nWidthOffset( 0 ), 
            m_nHeightOffset( 0 ),
            m_bCVARSOn( false )
            {}
#endif
        ////////////////////////////////////////////////////
    Figure( std::string sFigureName,
            const bool bColour = true ) 
        : m_sFigureName( sFigureName ), 
            m_bAllocated( false ),
            m_pDrawingBuffer( NULL ),
            m_bColour( bColour ),
            m_nWidthOffset( 0 ), 
            m_nHeightOffset( 0 ),
            m_bCVARSOn( false ) {
            cvInitFont( &m_Font, CV_FONT_HERSHEY_SIMPLEX, 1.0, 1.0, 0, 1, CV_AA );
        }

        ////////////////////////////////////////////////////
    Figure( std::string sFigureName, 
            const int nWidth, 
            const int nHeight,
            const bool bColour = true
            ) : 
        m_sFigureName( sFigureName ), 
            m_bAllocated( false ),
            m_pDrawingBuffer( NULL ),
            m_bColour( bColour ) ,
            m_nWidthOffset( 0 ), 
            m_nHeightOffset( 0 ),
            m_bCVARSOn( false ) {
            cvInitFont( &m_Font, CV_FONT_HERSHEY_SIMPLEX, 1.0, 1.0, 0, 1, CV_AA );
            _Allocate( nWidth, nHeight );
        }

        ////////////////////////////////////////////////////
        ~Figure() {
            if( m_bAllocated ) { cvReleaseImage( &m_pDrawingBuffer); }
            m_bAllocated = false;
            m_pDrawingBuffer = NULL;
            cvDestroyWindow( m_sFigureName.c_str() ); // Check for existence?
        }

        ////////////////////////////////////////////////////
        void imshow( unsigned char* pImageData,
                     const unsigned int nWidth,
                     const unsigned int nHeight,
                     const unsigned int nWidthStep,
                     const int nChannels = 1 ) {            
            _Allocate( nWidth,
                       nHeight );
            CvMat m;
            cvInitMatHeader( &m, nWidth, nHeight, nChannels == 1 ? CV_8UC1 : CV_8UC3,
                             pImageData, nWidthStep );
            IplImage stub, *pIm = NULL;
            pIm = cvGetImage( &m, &stub );
            imshow( pIm );
        }

        ////////////////////////////////////////////////////
        void imshow( IplImage* pImage ) {            
            _Allocate( pImage->width,
                       pImage->height );
            copy( pImage, m_pDrawingBuffer );            
        }

        ////////////////////////////////////////////////////
        void imshow2( unsigned char* pImageData1,
                      const unsigned int nWidth1,
                      const unsigned int nHeight1,
                      const unsigned int nWidthStep1,
                      const int nChannels1,
                      unsigned char* pImageData2,
                      const unsigned int nWidth2,
                      const unsigned int nHeight2,
                      const unsigned int nWidthStep2,
                      const int nChannels2,
                      const bool bHorizontal = true ) {
            CvMat m1;
            cvInitMatHeader( &m1, nWidth1, nHeight1, nChannels1 == 1 ? CV_8UC1 : CV_8UC3,
                             pImageData1, nWidthStep1 );
            CvMat m2;
            cvInitMatHeader( &m2, nWidth2, nHeight2, nChannels2 == 1 ? CV_8UC1 : CV_8UC3,
                             pImageData2, nWidthStep2 );
            IplImage stub1, stub2, *pIm1 = NULL, *pIm2 = NULL;
            pIm1 = cvGetImage( &m1, &stub1 );
            pIm2 = cvGetImage( &m2, &stub2 );
            imshow2( pIm1, pIm2, bHorizontal );
        }

        ////////////////////////////////////////////////////
        void imshow2( IplImage* pImage1, IplImage* pImage2, 
                      const bool bHorizontal = true ) {
            m_bHorizontal = bHorizontal;
            int nStereoImageWidth, nStereoImageHeight;
            StereoSize( pImage1, pImage2, bHorizontal, 
                        nStereoImageWidth, nStereoImageHeight );
            _Allocate( nStereoImageWidth, nStereoImageHeight );
            MakeStereo( pImage1, pImage2, &m_pDrawingBuffer,
                        bHorizontal, m_bColour );
            m_nWidthOffset = bHorizontal ? pImage1->width : 0;
            m_nHeightOffset = bHorizontal ? 0 : pImage1->height;
        }

        ////////////////////////////////////////////////////
        void imshow( IplImage* pImage, bool bKeepSize ) {
            if( !bKeepSize ) {
                imshow( pImage );
            }
            else {
                std::cout << "Not yet implemented!" << std::endl;
            }
        }

        ////////////////////////////////////////////////////
        template<class InputIterator, class Functor >
            void plot( InputIterator first, InputIterator last, 
                       Functor& reader, 
                       const std::string& sOptions,
                       const int nWidthOffset = 0,
                       const int nHeightOffset = 0
                       ) {
            if( !m_bAllocated ) {
                std::cout << "Not yet supported" << std::endl;
                return;
            }
            COPENCV::plot<InputIterator,Functor>
                ( m_pDrawingBuffer, first, last, reader,
                  sOptions, nWidthOffset, nHeightOffset );
        }

        ////////////////////////////////////////////////////
        template<class InputIterator1, class InputIterator2,
            class Functor1, class Functor2 >
            void plot2( InputIterator1 first1, InputIterator1 last1, Functor1& reader1, 
                        InputIterator2 first2, InputIterator2 last2, Functor2& reader2, 
                        const std::string& sOptions ) {
            if( !m_bAllocated ) {
                std::cout << "Not yet supported" << std::endl;
                return;
            }
            COPENCV::plot2<>( m_pDrawingBuffer, 
                              first1, last1, reader1,
                              first2, last2, reader2,
                              sOptions,
                              m_nWidthOffset, m_nHeightOffset );
        }

        ////////////////////////////////////////////////////
        template<class T>
        void plot( const std::vector<std::pair<T,T> >& vPoints, 
                   const std::string& sOptions,
                   const int nWidthOffset = 0,
                   const int nHeightOffset = 0 ) {
            plot<>( vPoints.begin(), vPoints.end(), pair_reader<T>, 
                    sOptions, nWidthOffset, nHeightOffset );
        }

        ////////////////////////////////////////////////////
        template<class T>
        void plot2( const std::vector<std::pair<T,T> >& vPoints1, 
                    const std::vector<std::pair<T,T> >& vPoints2, 
                    const std::string& sOptions ) {
            plot2<>( vPoints1.begin(), vPoints1.end(), pair_reader<T>, 
                     vPoints2.begin(), vPoints2.end(), pair_reader<T>, 
                     sOptions );
        }

        ////////////////////////////////////////////////////
        template<class T>
        void plot2( const std::vector<std::pair<T,T> >& vPoints, 
                    const std::string& sOptions ) {
            plot<>( vPoints.begin(), vPoints.end(), 
                    pair_reader<T>, sOptions,
                    m_nWidthOffset, m_nHeightOffset );
        }

        ////////////////////////////////////////////////////
        void text( const std::string& sMsg, 
                   const int nPosX, const int nPosY ) {
            if( m_bAllocated ) {
                cvPutText( m_pDrawingBuffer, 
                           sMsg.c_str(), 
                           cvPoint( nPosX, nPosY ),
                           &m_Font, cvScalar(255, 255, 255, 0) );
            }
        }

        ////////////////////////////////////////////////////
        void draw() {
            if( m_bAllocated ) {
                if( m_bCVARSOn ) {
                    _DrawCVARS();
                }
                cvShowImage( m_sFigureName.c_str(), m_pDrawingBuffer );
            }
        }

#ifndef COPENCV_HAS_CVARS
        ////////////////////////////////////////////////////
        int wait( const int nTimeMS = 0 ) {
            return cvWaitKey( nTimeMS );
        }
#else
        ////////////////////////////////////////////////////
        int wait( const int nTimeMS = 0 ) {
            int nKey = cvWaitKey( nTimeMS );
            // Catch specific key associations
            // (this could be made configurable...)
            if( (char)nKey == '~' ||
                (char)nKey == '`' ) {
                m_bCVARSOn = !m_bCVARSOn;
            }
            else if( m_bCVARSOn && nKey != -1 ) {
                if( (char)nKey == '\n' || // UK std keyboard
                    nKey == 10
                    ) { // Return
                    m_OpenCVConsole.NewCommand();
                }
                else if( (char) nKey == '\b' ) { // Backspace
                    m_OpenCVConsole.RemoveCharacter();
                }
                else if( (char)nKey == '\t' ||
                         nKey == 9
                         ) { // TAB completion
                    m_OpenCVConsole.TabComplete();
                }
                else if( nKey == 1113937 ||
                         nKey == 65361 ) { // left
                    m_OpenCVConsole.CursorLeft();
                }
                else if( nKey == 1113939 ||
                         nKey == 65363 ) { // right
                    m_OpenCVConsole.CursorRight();
                }
                else if( nKey == 1113938 ||
                         nKey == 65362 ) { // up
                    m_OpenCVConsole.CursorUp();
                }
                else if( nKey == 1113940 ||
                         nKey == 65364 ) { // down
                    m_OpenCVConsole.CursorDown();
                }
                else if( nKey == 1310821 ||
                         nKey == 262245 ) { // CTRL+e
                    m_OpenCVConsole.CursorEOF();
                }
                else if( nKey == 1310817 ||
                         nKey == 262241 ) { // CTRL+a
                    m_OpenCVConsole.CursorBOF();
                }
                else if( nKey == 1310819 ||
                         nKey == 262243) { // CTRL+c
                    m_OpenCVConsole.CursorClear();
                }
                else if( nKey == 1310827 ||
                         nKey == 262251 ) { // CTRL+k
                    m_OpenCVConsole.CursorKillEnd();
                }
                else if( isalnum( (char)nKey ) || 
                         (char)nKey == '.' || 
                         (char)nKey == ' ' || 
                         (char)nKey == '=' || 
                         (char)nKey == '[' || 
                         (char)nKey == ']' ) {
                    m_OpenCVConsole.AddCharacter( (char)nKey );
                }
                //std::cout << nKey << std::endl;
                return -1;
            }

            return nKey;
        }
#endif

    private:
        ////////////////////////////////////////////////////
        void _Allocate( const int nWidth,
                        const int nHeight ) {
            if( m_bAllocated && 
                ( nWidth != m_pDrawingBuffer->width ||
                  nHeight != m_pDrawingBuffer->height ) ) {
                cvReleaseImage( &m_pDrawingBuffer);
                m_pDrawingBuffer = NULL;
                m_bAllocated = false;
            }
            if( !m_bAllocated ) {
                m_bAllocated = true;
                m_pDrawingBuffer = cvCreateImage( cvSize( nWidth, nHeight ),
                                                  IPL_DEPTH_8U, m_bColour ? 3 : 1 );
            }
        }

#ifndef COPENCV_HAS_CVARS
        ////////////////////////////////////////////////////////////////////////
        void _DrawCVARS() {}
#else
        ////////////////////////////////////////////////////////////////////////
        void _DrawCVARS() {
            if( m_bAllocated ) {
                m_OpenCVConsole.SetImage( m_pDrawingBuffer );
                m_OpenCVConsole.Render();
            }
        }
#endif

    private:
        ////////////////////////////////////////////////////////////////////////
        std::string m_sFigureName;
        bool m_bAllocated;
        IplImage* m_pDrawingBuffer;
        CvFont m_Font;
        bool m_bColour;
        bool m_bHorizontal;
        int m_nWidthOffset, m_nHeightOffset;
        bool m_bCVARSOn;
#if COPENCV_HAS_CVARS
        OpenCVConsole m_OpenCVConsole;
#endif
    };
    ////////////////////////////////////////////////////////////////////////////////
}

#endif
