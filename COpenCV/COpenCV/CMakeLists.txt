CMAKE_MINIMUM_REQUIRED( VERSION 2.8 )

INCLUDE_DIRECTORIES( ${CMAKE_SOURCE_DIR} )

##############################################################################
SET( HDRS 
    COpenCV.h
    )

##############################################################################
INSTALL( FILES ${HDRS} DESTINATION ${CMAKE_INSTALL_PREFIX}/include/COpenCV )

