# ==================================================================================================
#
# Defines the following variables:
#   FFTW_FOUND          Boolean holding whether or not the FFTW3 library was found
#   FFTW_INCLUDE_DIRS   The FFTW3 include directory
#   FFTW_LIBRARIES      The FFTW3 library
#
# ==================================================================================================

# set possible install locations
set(FFTW_HINTS
        ${FFTW_ROOT}
        $ENV{FFTW_ROOT})

# set default search path
set(FFTW_PATHS /usr/local)

# find include dir
find_path(FFTW_INCLUDE_DIRS
        NAMES fftw3.h
        HITS ${FFTW_HINTS}
        PATHS ${FFTW_PATHS}
        PATH_SUFFIXES include
        DOC "FFTW3 include dir")

# find lib
find_library(FFTW_LIBRARIES
        NAMES fftw3 libfftw3
        HINTS ${FFTW_HINTS}
        PATHS ${FFTW_PATHS}
        PATH_SUFFIXES lib lib64
        DOC "FFTW3 lib")

# find or not
if (NOT FFTW_INCLUDE_DIRS)
    message(STATUS "Could not find 'fftw3.h' in default search path ${FFTW_PATHS}, please set FFTW_ROOT")
endif()
if (NOT FFTW_LIBRARIES)
    message(STATUS "Could not find 'libfftw3' in default search path ${FFTW_PATHS}, please set FFTW_ROOT")
endif()

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(FFTW
        REQUIRED_VARS FFTW_INCLUDE_DIRS
        HANDLE_COMPONENTS)

mark_as_advanced(
        FFTW_INCLUDE_DIRS
        FFTW_LIBRARIES)