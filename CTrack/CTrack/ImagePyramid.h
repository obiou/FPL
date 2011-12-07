// Copyright (C) 2010 Christopher Mei (christopher.m.mei@gmail.com)
//
// This file is part of the CTrack Library. This library is free
// software; you can redistribute it and/or modify it under the
// terms of the GNU Lesser General Public License as published by the
// Free Software Foundation; either version 2, or (at your option)
// any later version.

// This library is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU Lesser General Public License for more details.

// You should have received a copy of the GNU Lesser General Public
// License along with this library; see the file COPYING.LESSER. If
// not, write to the Free Software Foundation, 59 Temple Place - Suite
// 330, Boston, MA 02111-1307, USA.

// As a special exception, you may use this file as part of a free software
// library without restriction.  Specifically, if other files instantiate
// templates or use macros or inline functions from this file, or you compile
// this file and link it with other files to produce an executable, this
// file does not by itself cause the resulting executable to be covered by
// the GNU Lesser General Public License.  This exception does not however
// invalidate any other reasons why the executable file might be covered by
// the GNU Lesser General Public License.

#ifndef IMAGE_PYRAMID_H
#define IMAGE_PYRAMID_H

#include <assert.h>
#include <iostream>
#include <limits>
#include <math.h>
#include <vector>

#include <CTrack/Types.h>

namespace CTrack {
    ////////////////////////////////////////////////////////////////////////////
    /// Helper function.
    /// Returns the number of pyramid levels required to achieve a given size
    /// under the constraint that the sizes are dividable by 2^nNumLevels.
    unsigned int MinImageSizeToNumLevels( const int nWidth,
                                          const int nHeight,
                                          const int nMinWidth, ///< Input:
                                          const int nMinHeight, ///< Input:
                                          const bool bForcePowersOf2 = false  ///< Input: should we have 2^nNumLevels = nWidth or nHeight?
                                          )
    {
        // Compute min number of levels to achieve (without surpassing)
        // the image size
        double dMinRatio = std::min( (double) nWidth/nMinWidth, 
                                     (double) nHeight/nMinHeight );
        unsigned int nNumLevels  = int( floor( log( (double) dMinRatio  )/log(2) ) ) + 1;
        
        if( bForcePowersOf2 ) {
            // Compute maximal acceptable number of levels
            unsigned int nMaxLevels = (unsigned int) floor( std::min( log( (double) nWidth )/log(2.),
                                                                      log( (double) nHeight )/log(2.) ) );
            return std::min( nNumLevels, nMaxLevels );
        }
        else {
            return nNumLevels;
        }
    }

    ////////////////////////////////////////////////////////////////////////////
    class ImagePyramid {
    public:
        ImagePyramid() {}
        ~ImagePyramid() { _Free(); }

        void BuildPyramid( int nNumLevels, unsigned char* pImage, 
                           int nWidth, int nHeight, int nWidthStep );

        ImageHolder* GetImage() { return m_vImagePyramid.size() == 0 ? NULL : m_vImagePyramid[0]; }
        ImageHolder* GetImage( unsigned int nLevel ) { return nLevel >= m_vImagePyramid.size() ? NULL : m_vImagePyramid[nLevel]; }
        ImageHolder* GetSmallestImage() { return GetImage( m_vImagePyramid.size()-1 ); }

        unsigned int GetNumLevels() { return m_vImagePyramid.size(); }

    private:
        ImagePyramid( const ImagePyramid & );
        ImagePyramid& operator=( const ImagePyramid& );

    private:
        void _Free();
        bool _AllocatedWithCorrectSize( const int nWidth, 
                                        const int nHeight,
                                        const int nWidthStep );
        void _Allocate( const int nNumLevels,
                        unsigned char* pImage,
                        const int nWidth,
                        const int nHeight,
                        const int nWidthStep
                        );

    private:
        std::vector<ImageHolder*> m_vImagePyramid;
    };
}

////////////////////////////////////////////////////////////////////////////////
void CTrack::ImagePyramid::BuildPyramid( int nNumLevels, unsigned char* pImage, 
                                         int nWidth, int nHeight, int nWidthStep
                                         )
{    
    if( !_AllocatedWithCorrectSize( nWidth, nHeight, nWidthStep ) ) {
        _Free();
        _Allocate( nNumLevels, pImage, nWidth, nHeight, nWidthStep );
    }

    for( size_t nLevel = 1; nLevel < m_vImagePyramid.size(); nLevel++ ) {
#if USE_SSSE3
        if( IsWidthDividableBy16( vImagePyramid[ nLevel-1 ] ) ) {
            HalveImageSSSE3( m_vImagePyramid[ nLevel-1 ], m_vImagePyramid[ nLevel ] );
        }
        else {
            HalveImage( m_vImagePyramid[ nLevel-1 ], m_vImagePyramid[ nLevel ] );
        }
#else
        HalveImage( m_vImagePyramid[ nLevel-1 ]->pImageData, 
                    m_vImagePyramid[ nLevel-1 ]->nWidth, 
                    m_vImagePyramid[ nLevel-1 ]->nHeight, 
                    m_vImagePyramid[ nLevel-1 ]->nWidthStep, 
                    m_vImagePyramid[ nLevel ]->pImageData,
                    m_vImagePyramid[ nLevel ]->nWidthStep 
                    );
#endif
    }
}

////////////////////////////////////////////////////////////////////////////////
void CTrack::ImagePyramid::_Free()
{
    // Do not free image on first pyramid level
    for( size_t ii=1; ii < m_vImagePyramid.size(); ii++ ) {
        delete[] m_vImagePyramid[ii]->pImageData;
        delete( m_vImagePyramid[ii] );
    }
    m_vImagePyramid.clear();
}

////////////////////////////////////////////////////////////////////////////////
bool CTrack::ImagePyramid::_AllocatedWithCorrectSize( const int nWidth, 
                                                      const int nHeight,
                                                      const int nWidthStep )
{
    if( m_vImagePyramid.size() == 0 ) {
        return false;
    }
    return m_vImagePyramid[0]->nWidth == nWidth &&
        m_vImagePyramid[0]->nHeight == nHeight &&
        m_vImagePyramid[0]->nHeight == nWidthStep;
}

////////////////////////////////////////////////////////////////////////////////
void CTrack::ImagePyramid::_Allocate( const int nNumLevels,
                                      unsigned char* pImage, 
                                      const int nWidth,
                                      const int nHeight,
                                      const int nWidthStep
                                      )
{ 
    {
        ImageHolder* pImageHolder = new ImageHolder( pImage, nWidth, nHeight, nWidthStep );
        m_vImagePyramid.push_back( pImageHolder );
    }
    for( int nLevel = 1; nLevel < nNumLevels; nLevel++ ) {
        const int nLevelWidth  = nWidth >> nLevel;
        const int nLevelHeight = nHeight >> nLevel;
        unsigned char* pImage = new unsigned char[ nLevelWidth*nLevelHeight ];
        ImageHolder* pImageHolder = new ImageHolder( pImage, nLevelWidth, nLevelHeight, nLevelWidth );
        m_vImagePyramid.push_back( pImageHolder );
    }
}

////////////////////////////////////////////////////////////////////////////////

#endif
