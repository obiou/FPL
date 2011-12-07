################################################################################
# Executable targets
set( EXECUTABLES
  test_cgeom
)

################################################################################
set( LIBS ${CGEOM_LIBRARIES} )
set( INCLUDES ${CGEOM_INCLUDE_DIR} )
set( DEFINITIONS ${CGEOM_DEFINITIONS} )

################################################################################
find_package( OPENCV REQUIRED )
set( LIBS ${LIBS} ${OPENCV_LIBRARIES} )
set( INCLUDES ${INCLUDES} ${OPENCV_INCLUDE_DIR} )
set( DEFINITIONS ${DEFINITIONS} "-DHAS_OPENCV" )

################################################################################
include_directories( ${INCLUDES} )
add_definitions( ${DEFINITIONS} )

################################################################################
foreach( EXECUTABLE ${EXECUTABLES} )
  add_executable( ${EXECUTABLE} ${EXECUTABLE}.cpp )
  ADD_DEPENDENCIES( ${EXECUTABLE} cgeom )
  target_link_libraries( ${EXECUTABLE} ${LIBS} )
endforeach( EXECUTABLE )
