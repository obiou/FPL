################################################################################
# Executable targets
SET( EXECUTABLES
    ccamera_model_load
    ccamera_model_row_col_timing
    ccamera_model_test
    factory
    test_projections
    test_projections2
)

################################################################################
SET( LIBS ${CCAMERAMODEL_LIBRARIES} )
SET( INCLUDES ${CCAMERAMODEL_INCLUDE_DIR} )
SET( DEFINITIONS ${CCAMERAMODEL_DEFINITIONS} )

################################################################################
FIND_PACKAGE( MISC REQUIRED )
SET( INCLUDES ${INCLUDES} ${MISC_INCLUDE_DIR} )

################################################################################
FIND_PACKAGE( COPENCV REQUIRED )
SET( INCLUDES ${INCLUDES} ${COPENCV_INCLUDE_DIR} )
SET( LIBS ${LIBS} ${COPENCV_LIBRARIES} )
SET( DEFINITIONS ${DEFINITIONS} ${COPENCV_DEFINITIONS} )

################################################################################
FIND_PACKAGE( COPENCV REQUIRED )
SET( INCLUDES ${INCLUDES} ${COPENCV_INCLUDE_DIR} )
SET( LIBS ${LIBS} ${COPENCV_LIBRARIES} )
SET( DEFINITIONS ${DEFINITIONS} ${COPENCV_DEFINITIONS} )

################################################################################
INCLUDE_DIRECTORIES( ${INCLUDES} )
ADD_DEFINITIONS( ${DEFINITIONS} )

################################################################################
FOREACH( EXECUTABLE ${EXECUTABLES} )
  ADD_EXECUTABLE( ${EXECUTABLE} ${EXECUTABLE}.cpp )
  ADD_DEPENDENCIES( ${EXECUTABLE} ccameramodel )
  TARGET_LINK_LIBRARIES( ${EXECUTABLE} ${LIBS} )
ENDFOREACH( EXECUTABLE )
