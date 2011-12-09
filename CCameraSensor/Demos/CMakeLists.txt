PROJECT( CCAMERASENSOR_DEMOS )
################################################################################
SET( LIBS ${CCAMERASENSOR_LIBRARIES} )
SET( INCLUDE_DIRS ${CCAMERASENSOR_INCLUDE_DIR} )
SET( DEFINITIONS ${CCAMERASENSOR_DEFINITIONS} )

################################################################################
# Executable targets
SET( EXECUTABLES
    ccamera_sensor_test
    opencv_test
    stereo_test
    trigger_test
    ueye_test
)

################################################################################
option( BUILD_CAMERA_LOGGER "Build camera logger - depends on c++0x." OFF )
if( BUILD_CAMERA_LOGGER )
    set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x " )
    set( EXECUTABLES ${EXECUTABLES} ccamera_logger )
    set( LIBS ${LIBS} pthread )
endif()

################################################################################
FIND_PACKAGE( COPENCV REQUIRED )
MESSAGE( STATUS "COPENCV_INCLUDE_DIR (Sensor): " ${COPENCV_INCLUDE_DIR} )
SET( INCLUDE_DIRS ${INCLUDE_DIRS} ${COPENCV_INCLUDE_DIR} )
SET( LIBS ${LIBS} ${COPENCV_LIBRARIES} )
SET( DEFINITIONS ${DEFINITIONS}  ${COPENCV_DEFINITIONS} )

################################################################################
INCLUDE_DIRECTORIES( ${INCLUDE_DIRS} )
ADD_DEFINITIONS( ${DEFINITIONS} )

################################################################################
FOREACH( EXECUTABLE ${EXECUTABLES} )
  ADD_EXECUTABLE( ${EXECUTABLE} ${EXECUTABLE}.cpp )
  ADD_DEPENDENCIES( ${EXECUTABLE} ccamerasensor )
  TARGET_LINK_LIBRARIES( ${EXECUTABLE} ${LIBS} )
ENDFOREACH( EXECUTABLE )
