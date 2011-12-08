CMAKE_MINIMUM_REQUIRED( VERSION 2.6 )

##############################################################################
INCLUDE_DIRECTORIES( ${CGEOM_INCLUDE_DIR} )

##############################################################################
SET( SRCS 
    CGeom.cpp
  )
  
##############################################################################
SET( HDRS 
    CGeom.h
  )

##############################################################################
SET( CGEOM_LIBRARIES ${CGEOM_LIBRARIES} ${CGEOM_LIBRARY} PARENT_SCOPE )

##############################################################################
add_library( cgeom  SHARED ${SRCS}  )
target_link_libraries( cgeom  ${OPENCV_LIBRARIES}  )
get_target_property( CGEOM_LIBRARY cgeom LOCATION )

##############################################################################
#INSTALL( TARGETS cgeom ARCHIVE DESTINATION ${CMAKE_INSTALL_PREFIX}/lib )
install( FILES 
  ${HDRS} 
  DESTINATION 
  ${CMAKE_INSTALL_PREFIX}/include/CGEOM
  )