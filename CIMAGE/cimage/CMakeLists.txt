CMAKE_MINIMUM_REQUIRED( VERSION 2.6 )

##############################################################################
INCLUDE_DIRECTORIES( ${CIMAGE_INCLUDE_DIR} )

##############################################################################
SET( SRCS 
#    cimage.cpp
  )
  
##############################################################################
SET( HDRS 
#    cimage.h
  )

##############################################################################
#add_library( cimage SHARED ${SRCS}  )
#target_link_libraries( cimage  ${CIMAGE_LIBRARIES}  )

##############################################################################
#get_target_property( CIMAGE_LIBRARY cimage LOCATION )
#SET( CIMAGE_LIBRARIES ${CIMAGE_LIBRARIES} ${CIMAGE_LIBRARY} PARENT_SCOPE )

##############################################################################
#INSTALL( TARGETS cimage ARCHIVE DESTINATION ${CMAKE_INSTALL_PREFIX}/lib )
install( FILES 
  ${HDRS} 
  DESTINATION 
  ${CMAKE_INSTALL_PREFIX}/include/CIMAGE
  )
