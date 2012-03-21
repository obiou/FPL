################################################################################
# Executable targets
set( EXECUTABLES
  test_cgeom
  test_gen_points
)

################################################################################
set( LIBS ${CGEOM_LIBRARIES} )
set( INCLUDES ${CGEOM_INCLUDE_DIR} )
set( DEFINITIONS ${CGEOM_DEFINITIONS} )

################################################################################
find_package( OPENCV2 REQUIRED )
set( LIBS ${LIBS} ${OpenCV2_LIBRARIES} )
set( INCLUDES ${INCLUDES} ${OpenCV2_INCLUDE_DIR} )
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
