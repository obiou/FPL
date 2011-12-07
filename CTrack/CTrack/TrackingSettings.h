#ifndef CTRACK_TRACKING_SETTINGS_H

#define CTRACK_TRACKING_SETTINGS_H

#include <vector>
#include <CTrack/TrackBuffer.h>

namespace CTrack {    
    ////////////////////////////////////////////////////////////////////////////
    /// This class encapsulates the settings for tracking: type of
    /// tracking, number of iterations, deal with out-of-image, ...
    template< class TrackingFunctor, class MaskFunctorT >
        class TrackingSettings {
    public:
        /// Default constructor: should provide sensible settings
    TrackingSettings( const double dPatchBegX, const double dPatchBegY,
                      const int nPatchWidth, const int nPatchHeight, 
                      const int nPatchWidthStep, bool bRegularise = false ) :
        m_dPatchBegX( dPatchBegX ), m_dPatchBegY( dPatchBegY ),
            m_bRegularise( bRegularise ),
            m_nMaxNumIterations( 30 ), 
            m_MaskFctr( nPatchWidth, nPatchHeight ),
            m_TrackBuffer( nPatchWidth, nPatchHeight, nPatchWidthStep ) {
            if( TrackingFunctor::BLUR ) {
                m_TrackBuffer.AllocateBlurBuffers();
            }
        }
        
        double BegX() { return m_dPatchBegX; }
        double BegY() { return m_dPatchBegY; }
        bool Regularise() { return m_bRegularise; }
        void MaxNumIterations( int nMaxNumIterations ) { m_nMaxNumIterations = nMaxNumIterations; }
        int MaxNumIterations() { return m_nMaxNumIterations; }
        MaskFunctorT& GetMaskFctrRef() { return m_MaskFctr; }
        TrackBuffer* GetTrackBuffer() { return &m_TrackBuffer; }
    private:
        double m_dPatchBegX;
        double m_dPatchBegY;
        bool m_bRegularise;
        int m_nMaxNumIterations;
        MaskFunctorT m_MaskFctr;
        TrackBuffer m_TrackBuffer;
    };
}

#endif
