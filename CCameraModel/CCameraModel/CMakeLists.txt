CMAKE_MINIMUM_REQUIRED( VERSION 2.8 )

##############################################################################
INCLUDE_DIRECTORIES( ${CCAMERAMODEL_INCLUDE_DIR} )
ADD_DEFINITIONS( ${CCAMERAMODEL_DEFINITIONS} )

##############################################################################
SET( SRCS
    CameraModel.cpp
    CameraModelInterface.cpp
    CameraStringParsing.cpp
    GridCalibrator.cpp
)

##############################################################################
SET( HDRS 
    CameraModel.h
    GridCalibrator.h
    )

##############################################################################
ADD_LIBRARY( ccameramodel SHARED ${SRCS} ${HDRS} )
target_link_libraries( ccameramodel ${CCAMERAMODEL_LIBRARIES} )

##############################################################################
GET_TARGET_PROPERTY( CCAMERAMODEL_LIBRARY ccameramodel LOCATION )
SET( CCAMERAMODEL_LIBRARIES ${CCAMERAMODEL_LIBRARIES} ${CCAMERAMODEL_LIBRARY} PARENT_SCOPE )

##############################################################################
#INSTALL( TARGETS ccameramodel ARCHIVE DESTINATION ${CMAKE_INSTALL_PREFIX}/lib )
INSTALL( FILES ${HDRS} DESTINATION ${CMAKE_INSTALL_PREFIX}/include/CCameraModel )

