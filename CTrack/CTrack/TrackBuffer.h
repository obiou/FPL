#ifndef CTRACK_TRACK_BUFFER_H

#define CTRACK_TRACK_BUFFER_H

#include <vector>

namespace CTrack {    
    ////////////////////////////////////////////////////////////////////////////
    /// This class can be used optionally to handle memory buffers
    /// used for tracking to avoid repeated allocation/deallocation.
    class TrackBuffer {
    public:
    TrackBuffer( int nPatchWidth, int nPatchHeight, int nPatchWidthStep, bool bBlur = false ) :
        m_nPatchWidth( nPatchWidth ), m_nPatchHeight( nPatchHeight ) {
            m_pRefPatchGradX = new float[ nPatchWidth*nPatchHeight ];
            m_pRefPatchGradY = new float[ nPatchWidth*nPatchHeight ];

            m_pCurPatch      = new unsigned char[ nPatchWidthStep*nPatchHeight ];
            m_pCurPatchGradX = new float[ nPatchWidth*nPatchHeight ];
            m_pCurPatchGradY = new float[ nPatchWidth*nPatchHeight ];

            if( bBlur ) {
                m_pRefPatchWTGradX = new float[ nPatchWidth*nPatchHeight ];
                m_pRefPatchWTGradY = new float[ nPatchWidth*nPatchHeight ];
            }
            else {
                m_pRefPatchWTGradX = NULL;
                m_pRefPatchWTGradY = NULL;
            }
        }
        ~TrackBuffer() {
            delete[] m_pRefPatchGradX;
            delete[] m_pRefPatchGradY;
            delete[] m_pCurPatch;
            delete[] m_pCurPatchGradX;
            delete[] m_pCurPatchGradY;
            if( m_pRefPatchWTGradX != NULL ) {
                delete[] m_pRefPatchWTGradX;      
            }      
            if( m_pRefPatchWTGradY != NULL ) {
                delete[] m_pRefPatchWTGradY;
            }
        }
        float* RefPatchGradX() { return m_pRefPatchGradX; }
        float* RefPatchGradY() { return m_pRefPatchGradY; }
        float* RefPatchWTGradX() { return m_pRefPatchWTGradX; }
        float* RefPatchWTGradY() { return m_pRefPatchWTGradY; }
        unsigned char* CurPatch() { return m_pCurPatch; }
        float* CurPatchGradX() { return m_pCurPatchGradX; }
        float* CurPatchGradY() { return m_pCurPatchGradY; }
        void AllocateBlurBuffers() {
            if( m_pRefPatchWTGradX != NULL ) {
                delete[] m_pRefPatchWTGradX;
            }
            m_pRefPatchWTGradX = new float[ m_nPatchWidth*m_nPatchHeight ];
            if( m_pRefPatchWTGradY != NULL ) {
                delete[] m_pRefPatchWTGradY;
            }
            m_pRefPatchWTGradY = new float[ m_nPatchWidth*m_nPatchHeight ];
        }
    private:
        TrackBuffer( const TrackBuffer& );
        TrackBuffer& operator=( const TrackBuffer& );
    private:
        unsigned char* m_pCurPatch;
        int m_nPatchWidth; 
        int m_nPatchHeight;
        float* m_pRefPatchGradX;
        float* m_pRefPatchGradY;
        float* m_pRefPatchWTGradX;
        float* m_pRefPatchWTGradY;
        float* m_pCurPatchGradX;
        float* m_pCurPatchGradY;
    };
}

#endif
