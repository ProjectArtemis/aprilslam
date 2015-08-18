# Findgtsam.cmake - Find gogole test
# Modified from FindGlog.cmake by alexs.mac@gmail.com (Alex Stewart)
#
# Findgtsam.cmake - Find Google test library.
#
# This module defines the following variables:
#
# GTSAM_FOUND: TRUE iff gtsam is found.
# GTSAM_INCLUDE_DIRS: Include directories for gtsam.
# GTSAM_LIBRARIES: Libraries required to link gtsam.
#
# The following variables control the behaviour of this module:
#
# GTSAM_INCLUDE_DIR_HINTS: List of additional directories in which to
#                          search for gtsam includes, e.g: /foo/include.
# GTSAM_LIBRARY_DIR_HINTS: List of additional directories in which to
#                          search for gtsam libraries, e.g: /bar/lib.
#
# The following variables are also defined by this module, but in line with
# CMake recommended FindPackage() module style should NOT be referenced directly
# by callers (use the plural variables detailed above instead).  These variables
# do however affect the behaviour of the module via FIND_[PATH/LIBRARY]() which
# are NOT re-called (i.e. search for library is not repeated) if these variables
# are set with valid values _in the CMake cache_. This means that if these
# variables are set directly in the cache, either by the user in the CMake GUI,
# or by the user passing -DVAR=VALUE directives to CMake when called (which
# explicitly defines a cache variable), then they will be used verbatim,
# bypassing the HINTS variables and other hard-coded search locations.
#
# GTSAM_INCLUDE_DIR: Include directory for gtsam, not including the
#                    include directory of any dependencies.
# GTSAM_LIBRARY: gtsam library, not including the libraries of any
#                dependencies.

# Called if we failed to find gtsam or any of it's required dependencies,
# unsets all public (designed to be used externally) variables and reports
# error message at priority depending upon [REQUIRED/QUIET/<NONE>] argument.
macro(GTSAM_REPORT_NOT_FOUND REASON_MSG)
    unset(GTSAM_FOUND)
    unset(GTSAM_INCLUDE_DIRS)
    unset(GTSAM_LIBRARIES)
    # Make results of search visible in the CMake GUI if gtsam has not
    # been found so that user does not have to toggle to advanced view.
    mark_as_advanced(CLEAR GTSAM_INCLUDE_DIR GTSAM_LIBRARY)
    # Note <package>_FIND_[REQUIRED/QUIETLY] variables defined by FindPackage()
    # use the camelcase library name, not uppercase.
    if(Gtsam_FIND_QUIETLY)
        message(STATUS "Failed to find gtsam - " ${REASON_MSG} ${ARGN})
    elseif(Gtsam_FIND_REQUIRED)
        message(FATAL_ERROR "Failed to find gtsam - " ${REASON_MSG} ${ARGN})
    else()
        # Neither QUIETLY nor REQUIRED, use no priority which emits a message
        # but continues configuration and allows generation.
        message("-- Failed to find gtsam - " ${REASON_MSG} ${ARGN})
    endif()
endmacro(GTSAM_REPORT_NOT_FOUND)

# Search user-installed locations first, so that we prefer user installs
# to system installs where both exist.
list(APPEND GTSAM_CHECK_INCLUDE_DIRS
    /usr/local/include
    /usr/local/homebrew/include # Mac OS X
    /opt/local/var/macports/software # Mac OS X.
    /opt/local/include
    /usr/include)
list(APPEND GTSAM_CHECK_LIBRARY_DIRS
    /usr/local/lib
    /usr/local/homebrew/lib # Mac OS X.
    /opt/local/lib
    /usr/lib)

# Check general hints
if(GTSAM_HINTS AND EXISTS ${GTSAM_HINTS})
    set(GTSAM_INCLUDE_DIR_HINTS ${GTSAM_HINTS}/include)
    set(GTSAM_LIBRARY_DIR_HINTS ${GTSAM_HINTS}/lib)
endif()

set(GTSAM_INCLUDE_FILE gtsam/config.h)
# Search supplied hint directories first if supplied.
find_path(GTSAM_INCLUDE_DIR
    NAMES ${GTSAM_INCLUDE_FILE}
    PATHS ${GTSAM_INCLUDE_DIR_HINTS}
          ${GTSAM_CHECK_INCLUDE_DIRS}
    NO_DEFAULT_PATH)
if(NOT GTSAM_INCLUDE_DIR OR NOT EXISTS ${GTSAM_INCLUDE_DIR})
GTSAM_REPORT_NOT_FOUND("Could not find gtsam include directory, "
    "set GTSAM_INCLUDE_DIR to directory containing gtsam/config.h")
endif()

find_library(GTSAM_LIBRARY
    NAMES gtsam
    PATHS ${GTSAM_LIBRARY_DIR_HINTS}
          ${GTSAM_CHECK_LIBRARY_DIRS}
    NO_DEFAULT_PATH)
if(NOT GTSAM_LIBRARY OR NOT EXISTS ${GTSAM_LIBRARY})
GTSAM_REPORT_NOT_FOUND("Could not find gtsam library, "
    "set GTSAM_LIBRARY to full path to libgtsam.")
endif()

# Mark internally as found, then verify. GTSAM_REPORT_NOT_FOUND() unsets
# if called.
set(GTSAM_FOUND TRUE)

# Catch case when caller has set GTSAM_INCLUDE_DIR in the cache / GUI and
# thus FIND_[PATH/LIBRARY] are not called, but specified locations are
# invalid, otherwise we would report the library as found.
if(GTSAM_INCLUDE_DIR AND NOT EXISTS ${GTSAM_INCLUDE_DIR}/${GTSAM_INCLUDE_FILE})
GTSAM_REPORT_NOT_FOUND("Caller defined GTSAM_INCLUDE_DIR:"
    " ${GTSAM_INCLUDE_DIR} does not contain gtsam/config.h header.")
endif()

# TODO: This regex for gtsam library is pretty primitive, we use lowercase
#       for comparison to handle Windows using CamelCase library names, could
#       this check be better?
string(TOLOWER "${GTSAM_LIBRARY}" LOWERCASE_GTSAM_LIBRARY)
if(GTSAM_LIBRARY AND NOT "${LOWERCASE_GTSAM_LIBRARY}" MATCHES ".*gtsam[^/]*")
GTSAM_REPORT_NOT_FOUND("Caller defined GTSAM_LIBRARY: "
    "${GTSAM_LIBRARY} does not match gtsam.")
endif()

# Set standard CMake FindPackage variables if found.
if(GTSAM_FOUND)
    set(GTSAM_INCLUDE_DIRS ${GTSAM_INCLUDE_DIR})
    list(APPEND GTSAM_LIBRARIES ${GTSAM_LIBRARY} tbb)
endif()

# Handle REQUIRED / QUIET optional arguments.
include(FindPackageHandleStandardArgs)
FIND_PACKAGE_HANDLE_STANDARD_ARGS(Gtsam DEFAULT_MSG
    GTSAM_INCLUDE_DIRS GTSAM_LIBRARIES)

# Only mark internal variables as advanced if we found gtsam, otherwise
# leave them visible in the standard GUI for the user to set manually.
if(GTSAM_FOUND)
    mark_as_advanced(FORCE GTSAM_INCLUDE_DIR GTSAM_LIBRARY)
endif()
