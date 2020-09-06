find_package(PkgConfig)
if(PKG_CONFIG_FOUND)
  pkg_check_modules(PC_RTLSDR rtlsdr QUIET)
endif()

find_path(RTLSDR_INCLUDE_DIRS rtl-sdr.h
                              PATHS ${PC_RTLSDR_INCLUDEDIR})
find_library(RTLSDR_LIBRARIES rtlsdr
                              PATHS ${PC_RTLSDR_LIBDIR})

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(rtlsdr REQUIRED_VARS RTLSDR_LIBRARIES RTLSDR_INCLUDE_DIRS)

mark_as_advanced(RTLSDR_INCLUDE_DIRS RTLSDR_LIBRARIES)
