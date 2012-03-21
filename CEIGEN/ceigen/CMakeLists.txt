CMAKE_MINIMUM_REQUIRED( VERSION 2.6 )

##############################################################################
INCLUDE_DIRECTORIES( ${CEIGEN_INCLUDE_DIR} )

##############################################################################
SET( SRCS 
    ceigen_basics.cpp
    ceigen_image.cpp
    ceigen_vision.cpp    
    )

##############################################################################
if( OPENCV2_FOUND )
    set( SRCS ${SRCS} ceigen_opencv.cpp )
endif()
  
##############################################################################
SET( HDRS 
    ceigen_basics.h
    ceigen_image.h
    ceigen_vision.h
  )


##############################################################################
#ADD_LIBRARY( ceigen ${SRCS} )
ADD_LIBRARY( ceigen SHARED ${SRCS} )
target_link_libraries( ceigen ${CEIGEN_LIBRARIES}  )

##############################################################################
GET_TARGET_PROPERTY( CEIGEN_LIBRARY ceigen LOCATION )
SET( CEIGEN_LIBRARIES ${CEIGEN_LIBRARIES} ${CEIGEN_LIBRARY} PARENT_SCOPE )

##############################################################################
#INSTALL( TARGETS ceigen ARCHIVE DESTINATION ${CMAKE_INSTALL_PREFIX}/lib )
install( FILES 
  ${HDRS} 
  DESTINATION 
  ${CMAKE_INSTALL_PREFIX}/include/ceigen
  )

