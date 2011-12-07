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
#include <iostream>
#include <fstream>

#include <CCameraSensorIncludes.h>

#if CCAMERASENSOR_HAS_OPENCV
#  include <cv.h>
#  include <highgui.h>
#endif

#include <string.h>
#include <string>

#include <CCameraSensor/Image.h>

namespace CCameraImage {
    not_impl_except Image::not_impl;
    ////////////////////////////////////////////////////////////////////////////
    Image* Image::copy() {
        Image* pImage = new Image;
        *pImage = *this;
        unsigned char* pCopiedData = new unsigned char[size()];
        memcpy( pCopiedData, this->data(), size() );
        pImage->data( pCopiedData );
        return pImage;
    }

    ////////////////////////////////////////////////////////////////////////////
#if CCAMERASENSOR_HAS_OPENCV
    ////////////////////////////////////////////////////////////////////////////
    IplImage ToIplImage( const Image* pImage ) {
        if( pImage == NULL ) {
            std::cerr << "ERROR: in ToIplImage, NULL image." << std::endl;
            IplImage ret; ret.width = 0;
            return ret;
        }
        if( pImage->type() != GL_UNSIGNED_BYTE ) {
            std::cerr << "ERROR: in ToIplImage, unsupported type." << std::endl;
            throw Image::not_impl;
        }

        if( pImage->format() != GL_LUMINANCE &&
            pImage->format() != GL_RGB  ) {
            std::cerr << "ERROR: in ToIplImage, unsupported format." << std::endl;
            throw Image::not_impl;
        }
        IplImage* pI = 
            cvCreateImageHeader
            ( cvSize( pImage->width(), pImage->height() ), 
              IPL_DEPTH_8U, pImage->format() == GL_LUMINANCE ? 1 : 3 );
        IplImage ret = *pI;
        ret.imageData = (char*)pImage->data();
        ret.imageDataOrigin = (char*)pImage->data();
        cvReleaseImageHeader( &pI );
        return ret;
    }

    ////////////////////////////////////////////////////////////////////////////
    Image FromIplImage( const IplImage* pImage ) {
        Image ret;
        if( pImage == NULL ) {
            std::cerr << "ERROR: in FromIplImage, NULL image." << std::endl;
            return ret;
        }
        if( pImage->depth != IPL_DEPTH_8U ) {
            std::cerr << "ERROR: in FromIplImage, unsupported type." << std::endl;
            return ret;
        }

        if( pImage->nChannels != 1 &&
            pImage->nChannels != 3 ) {
            std::cerr << "ERROR: in FromIplImage, unsupported number of channels: " 
                      << pImage->nChannels << " (only 1 or 3 accepted)."
                      << std::endl;
            return ret;
        }

        ret.width( pImage->width );
        ret.height( pImage->height );
        ret.widthStep( pImage->widthStep );
        ret.format( pImage->nChannels == 1 ? GL_LUMINANCE : GL_RGB );
        ret.type( GL_UNSIGNED_BYTE );
        ret.data( (unsigned char*) pImage->imageData );
        return ret;
    }
#endif  

    ////////////////////////////////////////////////////////////////////////////
    bool Image::save( const std::string& sImageFileName, const std::string& sExtraInfoFileName ) {
#if CCAMERASENSOR_HAS_OPENCV
        IplImage aI = ToIplImage( this );
        cvSaveImage( sImageFileName.c_str(), &aI );
        //std::cerr << "Saving " << sImageFileName << " and " << sExtraInfoFileName << std::endl;
        if( sExtraInfoFileName != "" ) {
            std::ofstream oFile( sExtraInfoFileName.c_str() );
            oFile << "image.sensorID = " << sensorID() << std::endl;
            oFile << "image.cameraTime = " << cameraTime() << std::endl;
            oFile << "image.time = " << time() << std::endl;
            oFile.close();
        }
        return true;
#else
        std::cerr << "Image.h compiled without CCAMERASENSOR_HAS_OPENCV flag, saving capability unavailable" << std::endl;
        std::cerr << "when trying to save " << sImageFileName << " and " << sExtraInfoFileName << std::endl;
        return false;
#endif
    }

    ////////////////////////////////////////////////////////////////////////////
    bool Image::load( const std::string& sImageFileName, const std::string& sExtraInfoFileName ) {
#if CCAMERASENSOR_HAS_OPENCV
        IplImage* pImage = cvLoadImage( sImageFileName.c_str() );
        //std::cerr << "Loading " << sImageFileName << " and " << sExtraInfoFileName << std::endl;
        if( sExtraInfoFileName != "" ) {
            std::ofstream oFile( sExtraInfoFileName.c_str() );
            oFile << "image.sensorID = " << sensorID() << std::endl;
            oFile << "image.cameraTime = " << cameraTime() << std::endl;
            oFile << "image.time = " << time() << std::endl;
            oFile.close();
        }
        return true;
#else
        std::cerr << "Image.h compiled without CCAMERASENSOR_HAS_OPENCV flag, saving capability unavailable" << std::endl;
        std::cerr << "when trying to save " << sImageFileName << " and " << sExtraInfoFileName << std::endl;
        return false;
#endif
    }
}

