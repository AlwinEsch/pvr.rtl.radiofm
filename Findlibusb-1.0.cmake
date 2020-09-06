find_package(PkgConfig)
if(PKG_CONFIG_FOUND)
  pkg_check_modules(PC_LIBUSB_1 libusb-1.0 QUIET)
endif()

find_path(LIBUSB_1_INCLUDE_DIRS libusb.h
                              PATHS ${PC_LIBUSB_1_INCLUDEDIR}
                              PATH_SUFFIXES libusb-1.0)
find_library(LIBUSB_1_LIBRARIES usb-1.0
                              PATHS ${PC_LIBUSB_1_LIBDIR})

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(libusb-1.0 REQUIRED_VARS LIBUSB_1_LIBRARIES LIBUSB_1_INCLUDE_DIRS)

mark_as_advanced(LIBUSB_1_INCLUDE_DIRS LIBUSB_1_LIBRARIES)
