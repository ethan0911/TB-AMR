# Locate p4est library
# This module defines
# P4EST_LIBRARY, the name of the library to link against
# P4EST_FOUND, if false, do not try to link to p4est
# P4EST_INCLUDE_DIR, where to find p4est.h
#

find_path(P4EST_INCLUDE_DIR p4est.h
  HINTS
  ${p4est_DIR}
  $ENV{p4est_DIR}
  PATH_SUFFIXES include
  PATHS
  ~/Library/Frameworks
  /Library/Frameworks
  /usr/local/include/
  /usr/include/
)

find_library(P4EST_LIBRARY_TEMP p4est
  HINTS
  ${p4est_DIR}
  $ENV{p4est_DIR}
  PATH_SUFFIXES lib
  PATHS
  /usr/lib/
)

find_library(SC_LIBRARY_TEMP sc
  HINTS
  ${p4est_DIR}
  $ENV{p4est_DIR}
  PATH_SUFFIXES lib
  PATHS
  /usr/lib/
)

set(P4EST_FOUND "NO")
if(P4EST_LIBRARY_TEMP AND SC_LIBRARY_TEMP)
  # Set the final string here so the GUI reflects the final state.
  set(P4EST_LIBRARIES "${P4EST_LIBRARY_TEMP};${SC_LIBRARY_TEMP}" CACHE STRING "Where the P4EST Library can be found")
  # Set the temp variable to INTERNAL so it is not seen in the CMake GUI
  set(P4EST_LIBRARY_TEMP "${P4EST_LIBRARY_TEMP}" CACHE INTERNAL "")
  set(SC_LIBRARY_TEMP "${SC_LIBRARY_TEMP}" CACHE INTERNAL "")

  set(P4EST_FOUND "YES")
endif()

include(FindPackageHandleStandardArgs)

find_package_handle_standard_args(P4EST REQUIRED_VARS
  P4EST_LIBRARIES P4EST_INCLUDE_DIR)


