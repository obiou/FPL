################################################################################
# Executable targets
set( EXECUTABLES
  test_CIMAGE
)

################################################################################
set( LIBS ${CIMAGE_LIBRARIES} )
set( INCLUDES ${CIMAGE_INCLUDE_DIR} )
set( DEFINITIONS ${CIMAGE_DEFINITIONS} )

################################################################################
#find_package( OPENCV2 REQUIRED )
#set( LIBS ${LIBS} ${OPENCV2_LIBRARIES} )
#set( INCLUDES ${INCLUDES} ${OPENCV2_INCLUDE_DIR} )
#set( DEFINITIONS ${DEFINITIONS} "-DHAS_OPENCV" )

################################################################################
include_directories( ${INCLUDES} )
add_definitions( ${DEFINITIONS} )

################################################################################
foreach( EXECUTABLE ${EXECUTABLES} )
  add_executable( ${EXECUTABLE} ${EXECUTABLE}.cpp )
  target_link_libraries( ${EXECUTABLE} ${LIBS} )
endforeach( EXECUTABLE )
