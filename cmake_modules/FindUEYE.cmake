# For UEYE, see
#
# The following are set after configuration is done: 
#  UEYE_FOUND
#  UEYE_INCLUDE_DIR
#  UEYE_LIBRARIES
#

MACRO( DBG_MSG _MSG )
#    MESSAGE( STATUS "${CMAKE_CURRENT_LIST_FILE}(${CMAKE_CURRENT_LIST_LINE}):\n${_MSG}" )
ENDMACRO( DBG_MSG )

# typical root dirs of installations, exactly one of them is used
SET( UEYE_POSSIBLE_ROOT_DIRS
    "${UEYE_ROOT_DIR}"
    "$ENV{UEYE_ROOT_DIR}"  
    "$ENV{UEYE_DIR}"  
    "$ENV{UEYE_HOME}" 
    /usr/local
    /usr
    )

FIND_PATH( UEYE_INCLUDE_DIR
    NAMES 
    uEye.h
    PATHS ${UEYE_POSSIBLE_ROOT_DIRS} 
    )

DBG_MSG( "UEYE_INCLUDE_DIR=${UEYE_INCLUDE_DIR}" )

FIND_LIBRARY( UEYE_LIBRARIES
    NAMES ueye_api
    PATHS ${UEYE_POSSIBLE_ROOT_DIRS}  
    )

DBG_MSG( "UEYE_LIBRARIES=${UEYE_LIBRARIES}" )

IF( UEYE_LIBRARIES )
    SET( UEYE_FOUND ON )
ENDIF( UEYE_LIBRARIES )

IF( NOT UEYE_FOUND )
    IF( NOT UEYE_FIND_QUIETLY )
        IF( UEYE_FIND_REQUIRED )
            MESSAGE( FATAL_ERROR
                "UEYE required but some headers or libs not found. Please specify it's location by setting UEYE_ROOT_DIR")
        ELSE( UEYE_FIND_REQUIRED )
            MESSAGE( STATUS 
                "ERROR: UEYE was not found.")
        ENDIF( UEYE_FIND_REQUIRED )
    ENDIF( NOT UEYE_FIND_QUIETLY )
ELSE( NOT UEYE_FOUND )
    DBG_MSG( "Success" )
    MARK_AS_ADVANCED(
        UEYE_ROOT_DIR
        UEYE_INCLUDE_DIR
        UEYE_LIBRARIES
        )
ENDIF( NOT UEYE_FOUND )
