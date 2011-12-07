#ifndef CTRACK_TRACKING_STATS_PYR_H

#define CTRACK_TRACKING_STATS_PYR_H

#include <vector>

namespace CTrack {    
    ////////////////////////////////////////////////////////////////////////////
    /// This class handles statistics for pyramid tracking
    class TrackingStatsPyr {
    public:
        TrackingStatsPyr( int nNumLevels ) {
            m_vTrackingStats.resize( nNumLevels );
        }
        TrackingStats* GetTrackingStats( unsigned int nLevel ) {
            return nLevel >= m_vTrackingStats.size() ? NULL : &m_vTrackingStats[ nLevel ];
        }
    private:
        std::vector<TrackingStats> m_vTrackingStats;
    };
}

#endif
