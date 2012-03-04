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
include_directories( ${INCLUDES} )
add_definitions( ${DEFINITIONS} )

################################################################################
foreach( EXECUTABLE ${EXECUTABLES} )
  add_executable( ${EXECUTABLE} ${EXECUTABLE}.cpp )
  target_link_libraries( ${EXECUTABLE} ${LIBS} )
endforeach( EXECUTABLE )
