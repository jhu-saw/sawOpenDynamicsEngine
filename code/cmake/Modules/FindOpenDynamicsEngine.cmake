cmake_minimum_required(VERSION 2.6.0)

if (WIN32)
  
  find_path( ODE_INCLUDE_DIR ode/ode.h 
    PATHS ENV ODE_ROOT_PATH PATH_SUFFIXES include )

  find_library( ODE_LIBRARY NAMES ode 
    PATHS ENV ODE_ROOT_PATH PATH_SUFFIXES lib )

elseif (APPLE)
  # TODO?
elseif (UNIX)

  find_path( ODE_INCLUDE_DIR ode/ode.h )

  # On RedHat, the ode double-precision library is called "ode-double" and
  # the single-precision is called "ode" but on Debian/Ubuntu they are
  # called "ode" and "ode-sp", repsectively
  find_library( ODE_LIBRARY ode-double ode )

  if (ODE_LIBRARY STREQUAL "ODE_LIBRARY-NOTFOUND")
    find_library( ODE_LIBRARY ode )
  endif ()

endif ()

set( ODE_FOUND FALSE )

if( ODE_INCLUDE_DIR ) 
  if( ODE_LIBRARY )
    set( ODE_FOUND TRUE )
  endif( ODE_LIBRARY )
endif( ODE_INCLUDE_DIR )

mark_as_advanced( ODE_INCLUDE_DIR ODE_LIBRARY )



