#ifndef CMISC_H
#define CMISC_H

#include <errno.h>
#include <stdio.h>  /* defines FILENAME_MAX */
#include <sys/time.h>

#ifdef WINDOWS
    #include <direct.h>
    #define GetCurrentDir _getcwd
#else
    #include <unistd.h>
    #define GetCurrentDir getcwd
 #endif

#include <string>

#include <some_stats.h>

namespace CMISC {
    ////////////////////////////////////////////////////////////////////////////////
    std::string GetCWD() {
        char cCurrentPath[FILENAME_MAX];
        if( !GetCurrentDir( cCurrentPath, sizeof(cCurrentPath) ) ) {
            //return errno;
            return "";
        }
        cCurrentPath[sizeof(cCurrentPath) - 1] = '\0'; 
        return std::string( cCurrentPath );
    }

    ////////////////////////////////////////////////////////////////////////////////
    std::string AbsPath( const std::string& sPath ) {
        if( sPath.size() == 0 ) {
            return "";
        }
        if( sPath[0] == '/' ) {
            return sPath;
        }
        return GetCWD() + "/" + sPath;
    }

    ////////////////////////////////////////////////////////////////////////////////
    inline double Tic() {
        struct timeval tv;
        gettimeofday(&tv, NULL);
        return static_cast<double>( tv.tv_sec ) + 1e-6 * static_cast<double>(tv.tv_usec);
    }

    ////////////////////////////////////////////////////////////////////////////////
    inline double TocMS( double t0 ) {
        return ( Tic() - t0 )*1000.;
    }
}

#endif
