// CCameraSensor - framework for camera drivers
// Copyright (C) 2011 C. Mei
// 
// CCameraSensor is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published
// by the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// 
// CCameraSensor is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
// 
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
//
#ifndef CIMAGE_H
#define CIMAGE_H

#include <iostream>

#if CCAMERASENSOR_HAS_OPENCV
#  include <cv.h>
#  include <highgui.h>
#endif

#include <string>

#ifdef DOES_NOT_HAVE_GL
/* Data types */
#  define GL_BYTE                                 0x1400
#  define GL_UNSIGNED_BYTE                        0x1401
#  define GL_SHORT                                0x1402
#  define GL_UNSIGNED_SHORT                       0x1403
#  define GL_INT                                  0x1404
#  define GL_UNSIGNED_INT                         0x1405
#  define GL_FLOAT                                0x1406
#  define GL_2_BYTES                              0x1407
#  define GL_3_BYTES                              0x1408
#  define GL_4_BYTES                              0x1409
#  define GL_DOUBLE                               0x140A
/* Buffers, Pixel Drawing/Reading */
#  define GL_NONE                                 0x0
#  define GL_LEFT                                 0x0406
#  define GL_RIGHT                                0x0407
/*GL_FRONT                                      0x0404 */
/*GL_BACK                                       0x0405 */
/*GL_FRONT_AND_BACK                             0x0408 */
#  define GL_FRONT_LEFT                           0x0400
#  define GL_FRONT_RIGHT                          0x0401
#  define GL_BACK_LEFT                            0x0402
#  define GL_BACK_RIGHT                           0x0403
#  define GL_AUX0                                 0x0409
#  define GL_AUX1                                 0x040A
#  define GL_AUX2                                 0x040B
#  define GL_AUX3                                 0x040C
#  define GL_COLOR_INDEX                          0x1900
#  define GL_RED                                  0x1903
#  define GL_GREEN                                0x1904
#  define GL_BLUE                                 0x1905
#  define GL_ALPHA                                0x1906
#  define GL_LUMINANCE                            0x1909
#  define GL_LUMINANCE_ALPHA                      0x190A
#  define GL_ALPHA_BITS                           0x0D55
#  define GL_RED_BITS                             0x0D52
#  define GL_GREEN_BITS                           0x0D53
#  define GL_BLUE_BITS                            0x0D54
#  define GL_INDEX_BITS                           0x0D51
#  define GL_SUBPIXEL_BITS                        0x0D50
#  define GL_AUX_BUFFERS                          0x0C00
#  define GL_READ_BUFFER                          0x0C02
#  define GL_DRAW_BUFFER                          0x0C01
#  define GL_DOUBLEBUFFER                         0x0C32
#  define GL_STEREO                               0x0C33
#  define GL_BITMAP                               0x1A00
#  define GL_COLOR                                0x1800
#  define GL_DEPTH                                0x1801
#  define GL_STENCIL                              0x1802
#  define GL_DITHER                               0x0BD0
#  define GL_RGB                                  0x1907
#  define GL_RGBA                                 0x1908
#else
// If this include fails, set DOES_NOT_HAVE_GL
#  include <GL/gl.h>
#endif

namespace CCameraImage {
    class not_impl_except : public std::exception {
        virtual inline const char* what() const throw() {
            return "CCameraImage::Image, function or format not yet supported";
        }
    };

    class Image {
    public:
    Image() : m_nWidth(0), m_nHeight(0), m_nWidthStep(0), m_nType(GL_BYTE), m_nFormat(GL_NONE), 
            m_pData(NULL), m_nTime(0), m_sSensorID("") {}

        Image* copy();
        bool save( const std::string& sImageFileName, const std::string& sExtraInfoFileName = "" );
        bool load( const std::string& sImageFileName, const std::string& sExtraInfoFileName = "" );
        unsigned int width() const { return m_nWidth; }
        void width( const unsigned int w ) { m_nWidth = w; }
        unsigned int height() const { return m_nHeight; }
        void height( const unsigned int h ) { m_nHeight = h; }
        unsigned int widthStep() const { return m_nWidthStep; }
        void widthStep( const unsigned int ws ) { m_nWidthStep = ws; }
        unsigned int type() const { return m_nType; }
        void type( const unsigned int t ) { m_nType = t; }
        unsigned int format() const { return m_nFormat; }
        void format( const unsigned int f ) { m_nFormat = f; }
        unsigned char* data() const { return m_pData; }
        void data( unsigned char* const pD ) { m_pData = pD; }
        unsigned long long time() const { return m_nTime; }
        void time( const unsigned long long t ) { m_nTime = t; }
        unsigned long long cameraTime() const { return m_nCameraTime; }
        void cameraTime( const unsigned long long t ) { m_nCameraTime = t; }
        std::string sensorID() const { return m_sSensorID; }
        void sensorID( const std::string& s ) { m_sSensorID = s; }
        unsigned int format_size() { return format() == GL_LUMINANCE ? 1 : format() == GL_RGB ? 3 : throw not_impl; }
        unsigned int type_size() { return type() == GL_UNSIGNED_BYTE ? 8 : throw not_impl; }
        unsigned int size() { return widthStep()*height()*format_size()*type_size(); }
    private:
        unsigned int m_nWidth;      /// Image width in pixels.
        unsigned int m_nHeight;     /// Image height in pixels.
        unsigned int m_nWidthStep;  /// Size of aligned image row in bytes.
        unsigned int m_nType;       /// GL_BYTE, GL_UNSIGNED_BYTE, ...  
        unsigned int m_nFormat;     /// GL_COLOR_INDEX, GL_LUMINANCe, ...
        unsigned char* m_pData;     /// Pointer to the allocated data.
        unsigned long long m_nTime; /// System time at which the image was taken (often estimated) in micro-seconds.
        unsigned long long m_nCameraTime; /// Time on the camera device at which the image was taken (often more precise) in micro-seconds.
        std::string m_sSensorID;    /// Unique camera identifier. 
    public:
        static not_impl_except not_impl;
    };

#if CCAMERASENSOR_HAS_OPENCV
    ////////////////////////////////////////////////////////////////////////////
    IplImage ToIplImage( const Image* pImage );
    Image FromIplImage( const IplImage* pImage );
#endif
}

#endif
