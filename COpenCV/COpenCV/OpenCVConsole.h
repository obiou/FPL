#ifndef OPENCV_CONSOLE_H
#define OPENCV_CONSOLE_H

#include <cv.h>
#include <highgui.h>

#include <algorithm>
#include <deque>
#include <string>

#include <sys/time.h>

#include <CVars/CVar.h>

namespace COPENCV {
    enum LINE_TYPE {
        LINE_ERROR,
        LINE_COMMAND,
        LINE_VAR
    };

    class OpenCVConsole;
    
    typedef std::pair<std::string,LINE_TYPE> CMD_LINE;

    ////////////////////////////////////////////////////////////////////////////////
    /// This function returns a pointer to the very first GLConsole ever created.
    //  As there should only be one, this is ok.  
    //  This is a workaround for header only programming.
    inline OpenCVConsole* GetConsole( OpenCVConsole* pFirstConsole = NULL )
    {
        static OpenCVConsole* pSavedConsole = NULL; 
        if( pSavedConsole ){
            return pSavedConsole;
        }
        if( pFirstConsole == NULL ){
            // if pFirstConsole is NULL (e.g. user is asking for this first console), then 
            // pSavedConsole BETTER not also be NULL; 
            fprintf( stderr, "ERROR: GLConsole has not been initialized!\n" );
        }
        else{
            pSavedConsole = pFirstConsole;        
        }
        return pSavedConsole;
    }

    ////////////////////////////////////////////////////////////////////////////////
    /// This function calls "GetConsole" to set the static variable pSavedConsole so
    //  we can get access to the console globally.
    //  This is a workaround for header only programming.
    inline void SetConsole( OpenCVConsole* pFirstConsole ) {
        GetConsole( pFirstConsole );
    }

    ////////////////////////////////////////////////////////////////////////////////
    /// Help for functions and variables or just general help.
    inline bool ConsoleHelp( std::vector<std::string> *vArgs );
    inline bool ConsoleFind( std::vector<std::string> *vArgs );

    ////////////////////////////////////////////////////////////////////////////
    class OpenCVConsole {
    public:
        ////////////////////////////////////////////////////////////////////////
    OpenCVConsole() :
        m_nHeightPercentage( CVarUtils::CreateGetCVar<unsigned int>( "console.HeightPercentage", 80 ) ),
            m_nTransparencyPercentage( CVarUtils::CreateGetCVar<unsigned int>( "console.Transparency", 20 ) ),
            m_BackColor( CVarUtils::CreateGetCVar<CVarUtils::Color>( "console.BackColor", 
                                                                     CVarUtils::Color( 25, 60, 120, 120 ) ) ),
            m_nCursorUpIndex( 0 ),
            m_pDrawingBuffer( NULL ),
            m_dBlinkRate( CVarUtils::CreateGetCVar<double>( "console.BlinkRate", 1.5 ) ), 
            m_dLastTic( 0 ) {
            //cvInitFont( &m_Font, CV_FONT_HERSHEY_COMPLEX_SMALL, 1, 1, 0, 1, CV_AA );
            cvInitFont( &m_Font, CV_FONT_HERSHEY_PLAIN, 1, 1, 0, 1, CV_AA );
            // CV_FONT_HERSHEY_SIMPLEX, CV_FONT_HERSHEY_PLAIN,
            // CV_FONT_HERSHEY_DUPLEX, CV_FONT_HERSHEY_COMPLEX,
            // CV_FONT_HERSHEY_TRIPLEX, CV_FONT_HERSHEY_COMPLEX_SMALL,
            // CV_FONT_HERSHEY_SCRIPT_SIMPLEX, CV_FONT_HERSHEY_SCRIPT_COMPLEX,
            SetConsole( this );
            CVarUtils::CreateGetCVar( "help", ConsoleHelp, 
                                      "Gives help information about the console or more specifically about a CVar." );
            CVarUtils::CreateGetCVar( "find", ConsoleFind,
                                      "Searches for variables with that name." );
        }

        ////////////////////////////////////////////////////////////////////////
        void SetImage( IplImage* pDrawingBuffer ) {
            m_pDrawingBuffer = pDrawingBuffer;
        }

        ////////////////////////////////////////////////////////////////////////////////
        inline double _Tic() {
            struct timeval tv;
            gettimeofday(&tv, NULL);
            return static_cast<double>( tv.tv_sec ) + 1e-6 * static_cast<double>(tv.tv_usec);
        }
        
        ////////////////////////////////////////////////////////////////////////////////
        inline double _Toc( double t0 ) {
            return _Tic() - t0;
        }

        ////////////////////////////////////////////////////////////////////////////////
        inline bool _IsCursorOn() {
            double dElapsed = _Toc( m_dLastTic );
            if( dElapsed > ( 1.0 / m_dBlinkRate ) ) {
                m_dLastTic = _Tic();
                return true;
            }
            else if( dElapsed > 0.50*( 1.0 / m_dBlinkRate ) ) {
                return false;
            }
            else {
                return true;
            }
        }

        ////////////////////////////////////////////////////////////////////////
        void Render() {
            const int nRenderHeight = int( m_pDrawingBuffer->height*m_nHeightPercentage/100. );
            const unsigned char nGray = 
                static_cast<unsigned char>( (m_BackColor.r+m_BackColor.g+m_BackColor.b)*255./3. );
            for( int ii=0; ii<nRenderHeight; ii++ ) {
                for( int jj=0; jj<m_pDrawingBuffer->width; jj++ ) {
                    CvScalar s;
                    s = cvGet2D( m_pDrawingBuffer, ii, jj );
                    if( m_pDrawingBuffer->nChannels == 1 ) {
                        s.val[0] = 
                            ( (100.-m_nTransparencyPercentage)*nGray + 
                              m_nTransparencyPercentage*s.val[0])/100.;
                    }
                    else if( m_pDrawingBuffer->nChannels == 3 ) {
                        // BGR format
                        s.val[0] =
                            ( (100.-m_nTransparencyPercentage)*m_BackColor.b*255 + 
                              m_nTransparencyPercentage*s.val[0])/100.;
                        s.val[1] = 
                            ( (100.-m_nTransparencyPercentage)*m_BackColor.g*255 + 
                              m_nTransparencyPercentage*s.val[1])/100.;
                        s.val[2] = 
                            ( (100.-m_nTransparencyPercentage)*m_BackColor.r*255 + 
                              m_nTransparencyPercentage*s.val[2])/100.;
                    }
                    cvSet2D( m_pDrawingBuffer, ii, jj, s );
                }
            }

            // Draw text
            std::string sCommandLine = "> " + m_sCurrentCommandBeg + m_sCurrentCommandEnd;
            std::string sCursor = "> " + m_sCurrentCommandBeg;
            std::fill( sCursor.begin(), sCursor.end(), ' ' );
            sCursor[ sCursor.size()-1 ] = '_';

            const int nInterLine = 3;

            CvSize textSize;
            int nYmin;
            cvGetTextSize( sCommandLine.c_str(), &m_Font,  
                           &textSize, &nYmin );
            

            int nFullHeight = textSize.height+nInterLine;

            // Start by command line
            cvPutText( m_pDrawingBuffer, sCommandLine.c_str(), cvPoint( 0, nRenderHeight-nYmin ), 
                       &m_Font, cvScalar( 255, 255, 255 , 0) );
            if( _IsCursorOn() ) { // Draw cursor
                CvSize textSize2;
                cvGetTextSize( ( "> " + m_sCurrentCommandBeg ).c_str(), &m_Font,  
                               &textSize2, NULL );
                cvPutText( m_pDrawingBuffer, "_", cvPoint( textSize2.width, nRenderHeight-nYmin ), 
                           &m_Font, cvScalar( 255, 255, 255 , 0) );
            }
            std::deque<std::pair<std::string,LINE_TYPE> >::iterator it; int nHeight;
            // Start at the bottom of the box
            for( nHeight = nRenderHeight-nFullHeight,
                     it = m_dCommands.begin(); 
                 nHeight > 0 && it != m_dCommands.end(); nHeight -= nFullHeight, it++ ) {
                
                if( (*it).second == LINE_ERROR ) {
                    CvScalar colour = cvScalar(0,0,255);
                    cvPutText( m_pDrawingBuffer, ( (*it).first + " : error " ).c_str(), cvPoint( 0, nHeight-nYmin ), 
                               &m_Font, colour );
                }
                else if( (*it).second == LINE_COMMAND ) {
                    CvScalar colour = cvScalar(0,255,0);
                    cvPutText( m_pDrawingBuffer, (*it).first.c_str(), cvPoint( 0, nHeight-nYmin ), 
                               &m_Font, colour );
                }
                else {
                    CvScalar colour = cvScalar(255,255,0);
                    cvPutText( m_pDrawingBuffer, (*it).first.c_str(), cvPoint( 0, nHeight-nYmin ), 
                               &m_Font, colour );
                }
            }
        }

        ////////////////////////////////////////////////////////////////////////
        void AddCharacter( char nChar ) {
            m_sCurrentCommandBeg += nChar;
        }

        ////////////////////////////////////////////////////////////////////////
        void RemoveCharacter() {
            if( !m_sCurrentCommandBeg.empty() ) {
                m_sCurrentCommandBeg = 
                    m_sCurrentCommandBeg.substr( 0, m_sCurrentCommandBeg.size()-1 );
            }
        }

        ////////////////////////////////////////////////////////////////////////
        void CursorLeft() {
            if( !m_sCurrentCommandBeg.empty() ) {
                m_sCurrentCommandEnd = 
                    m_sCurrentCommandBeg.substr
                    ( m_sCurrentCommandBeg.size()-1, m_sCurrentCommandBeg.size() ) 
                    + m_sCurrentCommandEnd;
                m_sCurrentCommandBeg.erase( m_sCurrentCommandBeg.size()-1, 1 );
            }
        }

        ////////////////////////////////////////////////////////////////////////
        void CursorRight() {
            if( !m_sCurrentCommandEnd.empty() ) {
                m_sCurrentCommandBeg +=  m_sCurrentCommandEnd.substr( 0, 1 );
                m_sCurrentCommandEnd.erase( 0, 1 );
            }
        }

        ////////////////////////////////////////////////////////////////////////
        void CursorEOF() {
            if( !m_sCurrentCommandEnd.empty() ) {
                m_sCurrentCommandBeg +=  m_sCurrentCommandEnd;
                m_sCurrentCommandEnd = "";
            }
        }

        ////////////////////////////////////////////////////////////////////////
        void CursorBOF() {
            if( !m_sCurrentCommandBeg.empty() ) {
                m_sCurrentCommandEnd =  m_sCurrentCommandBeg + m_sCurrentCommandEnd;
                m_sCurrentCommandBeg = "";
            }
        }

        ////////////////////////////////////////////////////////////////////////
        void CursorClear() {
            m_sCurrentCommandBeg = "";
            m_sCurrentCommandEnd = "";
        }

        ////////////////////////////////////////////////////////////////////////
        void CursorKillEnd() {
            m_sCurrentCommandEnd = "";
        }

        ////////////////////////////////////////////////////////////////////////
        void CursorUp() {
            std::deque<std::pair<std::string,LINE_TYPE> >::iterator it = m_dCommands.begin();
            if( it == m_dCommands.end() ) { return; }
            unsigned int nIndex = 0;
            m_nCursorUpIndex++;
            while( ++nIndex < m_nCursorUpIndex && ++it != m_dCommands.end() ) {}
            if( it != m_dCommands.end() ) {
                m_sCurrentCommandBeg = "";
                m_sCurrentCommandEnd = "";
                m_sCurrentCommandBeg = (*it).first;
            }
            else {
                m_nCursorUpIndex--;
            }
        }

        ////////////////////////////////////////////////////////////////////////
        void CursorDown() {
            std::deque<std::pair<std::string,LINE_TYPE> >::iterator it = m_dCommands.begin();
            if( it == m_dCommands.end() ) { return; }            
            if( m_nCursorUpIndex == 0 ) { return; }
            unsigned int nIndex = 0;
            m_nCursorUpIndex--;
            while( ++nIndex < m_nCursorUpIndex && ++it != m_dCommands.end() ) {}
            if( it != m_dCommands.end() ) {
                m_sCurrentCommandBeg = "";
                m_sCurrentCommandEnd = "";
                m_sCurrentCommandBeg = (*it).first;
            }
            else {
                m_nCursorUpIndex++;
            }
        }

        ////////////////////////////////////////////////////////////////////////
        void TabComplete() {
            CvSize textSize;
            cvGetTextSize( "_" , &m_Font, &textSize, NULL );
            const unsigned int nMaxNumCharactersPerLine = m_pDrawingBuffer->width/textSize.width;
            std::vector<std::string> vResult;
            if( CVarUtils::TabComplete( nMaxNumCharactersPerLine,
                                        m_sCurrentCommandBeg,
                                        vResult ) ) {
                if( vResult.size() == 1 && vResult[0] != m_sCurrentCommandBeg && vResult[0] != "" ) {
                    m_dCommands.push_front
                        ( std::pair<std::string,LINE_TYPE>( vResult[0], LINE_COMMAND ) );
                }
                else {
                    for( size_t ii=0; ii<vResult.size(); ii++ ) {
                        if( _Trim( vResult[ii] ) != "" ) {
                            m_dCommands.push_front
                                ( std::pair<std::string,LINE_TYPE>( vResult[ii], LINE_COMMAND ) );
                        }
                    }
                }
            }
            //std::cout << sResult << std::endl;
        }
        
        ////////////////////////////////////////////////////////////////////////
        std::string& _Trim( std::string &str ) {
            str.erase( str.find_last_not_of( ' ' ) + 1 );
            str.erase( 0, str.find_first_not_of( ' ' ) );
            return str;
        }

        ////////////////////////////////////////////////////////////////////////
        void NewCommand() {
            if( m_dCommands.size() >= m_nMaxHistory ) {
                m_dCommands.pop_back();
            }

            // Process command
            std::string sResult; 
            bool bExecute = true;
            
            std::string sCommand = m_sCurrentCommandBeg + m_sCurrentCommandEnd;
            if( _Trim( sCommand ) == "" ) {
                return;
            }
            m_vStringOutputs.clear();
            if( CVarUtils::ProcessCommand( sCommand, sResult, bExecute ) ) {
                sCommand = sCommand.substr( 0, sCommand.find("=") );
                sCommand = _Trim( sCommand );

                // A bit of a hack, add string output to beginning of command
                // Functions can return values through this variable
                if( m_vStringOutputs.empty() ) {
                    m_dCommands.push_front
                        ( std::pair<std::string,LINE_TYPE>
                          ( sCommand + " = " + sResult, LINE_COMMAND ) );
                }
                else {
                    m_dCommands.push_front
                        ( std::pair<std::string,LINE_TYPE>
                          ( sCommand + " = " + sResult, LINE_COMMAND ) );
                    for( size_t ii=0; ii<m_vStringOutputs.size(); ii++ ) {
                        m_dCommands.push_front
                            ( std::pair<std::string,LINE_TYPE>
                              ( m_vStringOutputs[ii], LINE_COMMAND ) );
                    }
                }
            }
            else {
                m_dCommands.push_front
                    ( std::pair<std::string,LINE_TYPE>( sCommand, LINE_ERROR ) );
            }

            // Reset the line
            m_sCurrentCommandBeg = "";
            m_sCurrentCommandEnd = "";
            m_nCursorUpIndex = 0;
        }

        ////////////////////////////////////////////////////////////////////////
        inline bool Help( std::vector<std::string> *vArgs ) {
            if( vArgs != NULL && vArgs->size() != 0 ) {
                for( size_t ii = 0; ii < vArgs->size(); ii++ ) {
                    try {
                        if( CVarUtils::GetHelp( vArgs->at(ii) ).empty() ) {
                            m_vStringOutputs.push_back( "No help available." );
                        }
                        else {
                            m_vStringOutputs.push_back( CVarUtils::GetHelp( vArgs->at(ii) ) );
                        }
                    }
                    catch( CVarUtils::CVarException e ) {
                        m_vStringOutputs.push_back( "Unknown variable " + vArgs->at(ii) + "." );
                        return true;
                    }
                }
            }
            return true;
        }

        ////////////////////////////////////////////////////////////////////////
        inline bool Find( std::vector<std::string> *vArgs ) {
            if( vArgs != NULL && vArgs->size() > 0 ) {
                for( size_t ii=0; ii<vArgs->size(); ii++ ) {
                    std::vector<std::string> vCVarNames = g_pCVarTrie->FindListSubStr( vArgs->at(ii) );          
                    for( size_t jj=0; jj<vCVarNames.size(); jj++ ) { 
                        m_vStringOutputs.push_back( vCVarNames[jj] );       
                    }
                }
            }
            return true;
        }

        ////////////////////////////////////////////////////////////////////////
    private:
        unsigned int& m_nHeightPercentage;   
        unsigned int& m_nTransparencyPercentage;   
        CVarUtils::Color& m_BackColor;
        unsigned int m_nCursorUpIndex;
        CvFont m_Font;            
        IplImage* m_pDrawingBuffer;
        std::deque<std::pair<std::string,LINE_TYPE> > m_dCommands;
        std::string m_sCurrentCommandBeg;
        std::string m_sCurrentCommandEnd;
        unsigned int m_nMaxHistory;
        double &m_dBlinkRate, m_dLastTic;
        std::vector<std::string> m_vStringOutputs;
    };   

    ////////////////////////////////////////////////////////////////////////////
    inline bool ConsoleHelp( std::vector<std::string> *vArgs ) {
        OpenCVConsole* pConsole = GetConsole();
        return pConsole->Help( vArgs );
    }

    ////////////////////////////////////////////////////////////////////////////
    inline bool ConsoleFind( std::vector<std::string> *vArgs ) {
        OpenCVConsole* pConsole = GetConsole();
        return pConsole->Find( vArgs );
    }
}

#endif
