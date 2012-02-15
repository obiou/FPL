################################################################################
SET( INCLUDE_DIRS ${CTRACK_INCLUDE_DIR} )
SET( LIBS         ${CTRACK_LIBRARIES} )
SET( DEFINITIONS  ${CTRACK_DEFINITIONS} )

################################################################################
# Executable targets
SET( EXECUTABLES
    ctrack_ccamera_model
    ctrack_cv
#    ctrack_test_basic  
#    ctrack_test_blur  
#    ctrack_test_logm
#    ctrack_test_halve
#    ctrack_test_updates  
#    ctrack_test_pyr   
    ctrack_test_tracking
    ctrack_test_tracking_pyr
    ctrack_test_tracking_blur
#    ctrack_test_tracking_opencv
#    ctrack_test_warp 
)

################################################################################
find_package( OPENCV2 REQUIRED )
SET( LIBS ${LIBS} ${OpenCV2_LIBRARIES} )
SET( INCLUDE_DIRS ${INCLUDE_DIRS} ${OpenCV2_INCLUDE_DIR} )

################################################################################
find_package( CCAMERASENSOR REQUIRED )
SET( LIBS ${LIBS} ${CCAMERASENSOR_LIBRARIES} )
SET( INCLUDE_DIRS ${INCLUDE_DIRS} ${CCAMERASENSOR_INCLUDE_DIR} )

################################################################################
#find_package( CFigs REQUIRED )
#include_directories( ${CFIGS_INCLUDE_DIR} )

################################################################################
INCLUDE_DIRECTORIES( ${INCLUDE_DIRS} )
ADD_DEFINITIONS( ${DEFINITIONS} )

################################################################################
#message( STATUS "LIBS: " ${LIBS} )

################################################################################
FOREACH( EXECUTABLE ${EXECUTABLES} )
    ADD_EXECUTABLE( ${EXECUTABLE} ${EXECUTABLE}.cpp )
    ADD_DEPENDENCIES( ${EXECUTABLE} ctrack )
    TARGET_LINK_LIBRARIES( ${EXECUTABLE} ${LIBS} )
ENDFOREACH( EXECUTABLE )
