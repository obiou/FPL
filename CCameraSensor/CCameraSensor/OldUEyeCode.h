#ifndef CCAMERA_UEYE_H
#define CCAMERA_UEYE_H

namespace CCameraSensor {
    class UEyeCamera;
    ////////////////////////////////////////////////////////////////////////////
    /// Functor used to compute the offset between the camera time and the
    /// system time
    class OffsetFunctor {
    public:
    OffsetFunctor( UEyeCamera* pUEyeCamera ) : m_nOffset( 0 ), 
            m_pUEyeCamera( pUEyeCamera ) {}
        /// This computes the offset between the camera time and the system time.
        /// This operator should be called using the event IS_SET_EVENT_FRAME, which
        /// is when the latest image arrives.
        /// We can then find the offset between this 'arrival' time and the camera
        /// time by looking at the image info which contains the camera time of acquisition.
        void operator()();
    private:
        unsigned long long m_nOffset;
        UEyeCamera* m_pUEyeCamera;
        struct timeval tv;
    };

    ////////////////////////////////////////////////////////////////////////////    
    /// Class used to compute the camera time to system time offset
    /// Similar to EventThread in QuEyeSdiDem but uses pthreads.
    template<class Functor>
        class UEyeEventThread {
    public:
    UEyeEventThread( UEyeCamera* pUEyeCamera )
        : m_bRunning(false), m_Functor( pUEyeCamera ),
            m_hCamera( (HIDS) 0 ), m_Event( 0 ) {
#ifndef __LINUX__
            m_hEvent = 0;
#endif
        }
 
        ~UEyeEventThread() {}
 
        // Create the thread and start work
        bool start( HIDS hCamera, int Event ) {
            if( m_bRunning ) return false;

            m_hCamera = hCamera;
            m_Event   = Event;
#ifndef __LINUX__
            if( is_InitEvent( m_hCamera, m_hEvent, m_Event ) != IS_SUCCESS ) {
                std::cerr << "ERROR: in UEyeCamera::run(), problem when calling is_InitEvent." << std::endl;
                return false;
            }
#endif
            if( is_EnableEvent( m_hCamera, m_Event ) != IS_SUCCESS ) {
                std::cerr << "ERROR: in UEyeCamera::open(), problem when calling is_EnableEvent." << std::endl;
                return false;
            }

            m_bRunning = true;
            std::cout << "Starting thread" << std::endl;
            pthread_create( &m_thread, 0, &UEyeEventThread::start_thread, this );
            return true;
        }
 
        void stop() {
            if( !m_bRunning ) return;
            m_bRunning = false;
            pthread_join( m_thread, 0 );
        }

    private:
        static void* start_thread( void *obj ) {
            reinterpret_cast<UEyeEventThread*>( obj )->do_work();
            return NULL;
        }
  
        void do_work() { 
            //std::cout << "do_work()" << std::endl;
            while( m_bRunning ) {
#ifdef __LINUX__
                if( is_WaitEvent( m_hCamera, m_Event, 1 ) == IS_SUCCESS ) {
                    //std::cout << "Calling functor." << std::endl;
                    m_Functor();
                }
#else
                if( WaitForSingleObject( m_hEvent, 1000 ) == WAIT_OBJECT_0 ) {
                    m_Functor();
                }
#endif
            }
            is_DisableEvent( m_hCamera, m_Event );
#ifndef __LINUX__
            is_ExitEvent( m_hCamera, m_Event );
#endif            
        }                    
 
    private:
        volatile bool m_bRunning;
        pthread_t m_thread;
        Functor m_Functor;
        
        // Camera handle for waiting event
        HIDS m_hCamera;
        // Event waiting for
        int m_Event;
#ifndef __LINUX__
        // On windows we need an Event handle member
        HANDLE m_hEvent;
#endif
    };

    ////////////////////////////////////////////////////////////////////////////    
    ////////////////////////////////////////////////////////////////////////////    
    ////////////////////////////////////////////////////////////////////////////    
    void OffsetFunctor::operator()() { 
        if( m_pUEyeCamera == NULL ) return;        
        // Get current system time
        
        gettimeofday(&tv, NULL);
        unsigned long long nArrivalTime =  1e6*tv.tv_sec + (tv.tv_usec);
             
        // Get camera time from this image
        // Hoping here that the locking mechanism works well :-/
        // We devide by 10 to use the same units
        unsigned long long nAcquisitionCameraTime = m_pUEyeCamera->GetLatestImageCameraTime()/10;

        std::cout << "Offset operator" << std::endl;
        std::cout << nArrivalTime << std::endl;
        std::cout << nAcquisitionCameraTime << std::endl;
        std::cout << "Com. offset: " << nArrivalTime - nAcquisitionCameraTime << std::endl;
    }
}

#endif
