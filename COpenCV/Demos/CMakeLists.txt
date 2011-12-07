################################################################################
# Executable targets
SET( EXECUTABLES
    copencv_test
    copencv_imshow
)

################################################################################
SET( LIBS ${COPENCV_LIBRARIES} )
SET( INCLUDE_DIRS ${COPENCV_INCLUDE_DIR} )
SET( DEFINITIONS  ${COPENCV_DEFINITIONS} )

################################################################################
FIND_PACKAGE( CCAMERASENSOR REQUIRED )
IF( CCAMERASENSOR_FOUND )
  SET( LIBS ${LIBS} ${CCAMERASENSOR_LIBRARIES} )
  SET( INCLUDE_DIRS ${INCLUDE_DIRS} ${CCAMERASENSOR_INCLUDE_DIR} )
  SET( DEFINITIONS ${DEFINITIONS} ${CCAMERASENSOR_DEFINITIONS} )
ENDIF( CCAMERASENSOR_FOUND )

message( STATUS "INCLUDE_DIRS: " ${INCLUDE_DIRS} )

################################################################################
INCLUDE_DIRECTORIES( ${INCLUDE_DIRS} )
ADD_DEFINITIONS( ${DEFINITIONS} )

################################################################################
FOREACH( EXECUTABLE ${EXECUTABLES} )
  ADD_EXECUTABLE( ${EXECUTABLE} ${EXECUTABLE}.cpp )
  add_dependencies( ${EXECUTABLE} ${LIBS} )
  TARGET_LINK_LIBRARIES( ${EXECUTABLE} ${LIBS} )
ENDFOREACH( EXECUTABLE )
