CMAKE_MINIMUM_REQUIRED( VERSION 2.6 )

##############################################################################
SET( INCLUDE_DIRS ${CTRACK_INCLUDE_DIR} )
INCLUDE_DIRECTORIES( ${INCLUDE_DIRS} )

##############################################################################
SET( SRCS 
  Homography.cpp
  Jacobians.cpp
  MatrixRoot.cpp
  #MatrixLogarithm.cpp
  )
  
##############################################################################
SET( HDRS 
  Homography.h
  Jacobians.h
  Masks.h
  TrackingFunctors.h
  Transforms.h
  TransformsHighLevel.h
  )

##############################################################################
ADD_LIBRARY( ctrack SHARED ${SRCS} )
GET_TARGET_PROPERTY( CTRACK_LIBRARY ctrack LOCATION )
SET( CTRACK_LIBRARIES ${CTRACK_LIBRARIES} ${CTRACK_LIBRARY} PARENT_SCOPE )

##############################################################################
#INSTALL( TARGETS ctrack ARCHIVE DESTINATION ${CMAKE_INSTALL_PREFIX}/lib )
install( FILES 
  ${HDRS} 
  DESTINATION 
  ${CMAKE_INSTALL_PREFIX}/include/CTrack 
  )

