# - Try to find a package and add the *_INCLUDE_DIR, *_LIBRARIES, *_DEFINITIONS
# to the package with the second argument name
#
# Variable:
#   PCHSupport_FOUND
#
# Macro:
#   find_package_and_add

MACRO( find_package_and_add ) #_packageToFind _packageToAdd _packageDef )
    set( _packageToFind ${ARGV0} )
    set( _packageToAdd ${ARGV1} )


    if( NOT _packageToFind OR NOT _packageToAdd ) 
        message( FATAL_ERROR "USAGE: find_package_and_add( packageToFind packageToAdd [DefinitionToAdd] )" )
    endif()

    find_package( ${_packageToFind} REQUIRED )

    # Convention: include and libraries names are in uppercase
    string( TOUPPER ${_packageToFind} _packageToFind )

    set( get_inc "${_packageToFind}_INCLUDE_DIR" )
    set( get_lib "${_packageToFind}_LIBRARIES" )
    set( get_lib2 "${_packageToFind}_LIBRARY" )
    set( get_def "${_packageToFind}_DEFINITIONS" )
    
    set( add_inc "${_packageToAdd}_INCLUDE_DIR" )
    set( add_lib "${_packageToAdd}_LIBRARIES" )
    set( add_def "${_packageToAdd}_DEFINITIONS" )

    set( ${add_inc} ${${add_inc}} ${${get_inc}} )
    set( ${add_lib} ${${add_lib}} ${${get_lib}} )
    set( ${add_lib} ${${add_lib}} ${${get_lib2}} )
    set( ${add_def} ${${add_def}} ${${get_def}} )
    
    if( ${ARGV3} )
        set( _packageDef ${ARGV4} )
        #message( STATUS "_packageDef: " ${_packageDef} )
        set( ${add_def} ${${add_def}} ${_packageDef} )
    endif()
ENDMACRO( find_package_and_add )
