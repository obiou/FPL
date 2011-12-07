#ifndef CTRACK_TRACKING_SETTINGS_PYR_H

#define CTRACK_TRACKING_SETTINGS_PYR_H

#include <vector>

#include <CTrack/TrackingSettings.h>

namespace CTrack {    
    ////////////////////////////////////////////////////////////////////////////
    /// This class encapsulates the settings for tracking using pyramids: type of
    /// tracking, number of iterations, deal with out-of-image, ...
    template< class TrackingFunctor, class MaskFunctorT >
        class TrackingSettingsPyr {
    public:
        TrackingSettingsPyr()  {}
        ~TrackingSettingsPyr() { _Free(); };
        bool InitReset( const double dPatchBegX, const double dPatchBegY,
                        const int nPatchWidth, const int nPatchHeight, 
                        const int nPatchWidthStep, const bool bRegularise = false,
                        const int nNumLevels = 1 ) {
            _Free();
            // WidthStep is only used at the heighest level
            m_vTrackingSettings.push_back
                ( new TrackingSettings<TrackingFunctor,MaskFunctorT>
                  ( dPatchBegX, dPatchBegY, nPatchWidth, nPatchHeight, nPatchWidthStep, bRegularise ) );
            for( int nLevel=1; nLevel<nNumLevels; nLevel++ ) {
                m_vTrackingSettings.push_back
                    ( new TrackingSettings<TrackingFunctor,MaskFunctorT>
                      ( dPatchBegX/(1 << nLevel), dPatchBegY/(1 << nLevel),
                        nPatchWidth >> nLevel, nPatchHeight >> nLevel, 
                        nPatchWidth >> nLevel, bRegularise ) );
            }
            return true;
        }
        unsigned int GetNumLevels() { return m_vTrackingSettings.size(); }
        TrackingSettings<TrackingFunctor,MaskFunctorT>* GetTrackingSettings( unsigned int nLevel ) { 
            return nLevel >= m_vTrackingSettings.size() ? NULL : m_vTrackingSettings[nLevel]; }
        TrackBuffer* GetTrackBuffer() { return m_vTrackingSettings[0]->GetTrackBuffer(); }
    private:
        void _Free() {
            for( size_t ii=0; ii<m_vTrackingSettings.size(); ii++ ) {
                delete m_vTrackingSettings[ii];
            }
            m_vTrackingSettings.clear();
        }
    private:
        std::vector<TrackingSettings<TrackingFunctor,MaskFunctorT>* > m_vTrackingSettings;
    };
}

#endif
