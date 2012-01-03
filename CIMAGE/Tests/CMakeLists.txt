################################################################################
# Test executable targets
set( TEST_EXECUTABLES
  test_read_write
)

################################################################################
set( LIBS ${CIMAGE_LIBRARIES} )
set( INCLUDES ${CIMAGE_INCLUDE_DIR} )
set( DEFINITIONS ${CIMAGE_DEFINITIONS} )

################################################################################
include_directories( ${INCLUDES} )
add_definitions( ${DEFINITIONS} )

################################################################################
foreach( EXECUTABLE ${TEST_EXECUTABLES} )
  add_executable( ${EXECUTABLE} ${EXECUTABLE}.cpp )
  ADD_DEPENDENCIES( ${EXECUTABLE} cimage )
  target_link_libraries( ${EXECUTABLE} ${LIBS} )
  add_test( NAME ${EXECUTABLE} COMMAND ${CMAKE_BINARY_DIR}/CIMAGE/Tests/${EXECUTABLE} )
endforeach( EXECUTABLE )
