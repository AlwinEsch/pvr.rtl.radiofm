# - Try to find librtlsdr
# Once done this will define
#
# RTL_SDR_FOUND - system has librtlsdr
# RTL_SDR_INCLUDE_DIRS - the librtlsdr include directory
# RTL_SDR_LIBRARIES - The librtlsdr libraries

if(PKG_CONFIG_FOUND)
pkg_check_modules (RTLSDR rtlsdr)
list(APPEND RTL_SDR_INCLUDE_DIRS ${RTL_SDR_INCLUDEDIR})
endif()
if(NOT RTL_SDR_FOUND)
find_path( 	RTL_SDR_INCLUDE_DIRS "rtl-sdr.h"
			PATH_SUFFIXES "rtlsdr" )
			
find_library( 	RTL_SDR_LIBRARIES
				NAMES "rtlsdr"
				PATH_SUFFIXES "rtlsdr" )
endif()
# handle the QUIETLY and REQUIRED arguments and set RTL_SDR_FOUND to TRUE if
# all listed variables are TRUE
include( "FindPackageHandleStandardArgs" )
find_package_handle_standard_args(RtlSdr DEFAULT_MSG RTL_SDR_INCLUDE_DIRS RTL_SDR_LIBRARIES)

mark_as_advanced(RTL_SDR_INCLUDE_DIRS RTL_SDR_LIBRARIES)