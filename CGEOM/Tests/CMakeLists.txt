################################################################################
# Test executable targets
set( TEST_EXECUTABLES
  test_homog
)

################################################################################
set( LIBS ${CGEOM_LIBRARIES} )
set( INCLUDES ${CGEOM_INCLUDE_DIR} )
set( DEFINITIONS ${CGEOM_DEFINITIONS} )

################################################################################
include_directories( ${INCLUDES} )
add_definitions( ${DEFINITIONS} )

################################################################################
foreach( EXECUTABLE ${TEST_EXECUTABLES} )
  add_executable( ${EXECUTABLE} ${EXECUTABLE}.cpp )
  ADD_DEPENDENCIES( ${EXECUTABLE} cgeom )
  target_link_libraries( ${EXECUTABLE} ${LIBS} )
  add_test( NAME ${EXECUTABLE} COMMAND ${CMAKE_BINARY_DIR}/CGEOM/Tests/${EXECUTABLE} )
endforeach( EXECUTABLE )
