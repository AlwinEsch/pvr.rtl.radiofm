cmake_minimum_required(VERSION 3.5)
project(pvr.rtl.radiofm)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR})

find_package(rtlsdr REQUIRED)
find_package(TinyXML REQUIRED)
find_package(Kodi REQUIRED)
find_package(libusb-1.0 REQUIRED)

include_directories(${rtlsdr_INCLUDE_DIRS}
                    ${KODI_INCLUDE_DIR}
                    ${TINYXML_INCLUDE_DIRS}
                    ${LIBUSB_1_INCLUDE_DIRS})

set(BASIC_SOURCES src/RadioReceiver.cpp
                  src/RTL_SDR_Source.cpp
                  src/Filter.cpp
                  src/FirFilter.cpp
                  src/IirFilter.cpp
                  src/FmDecode.cpp
                  src/DownConvert.cpp
                  src/FreqShift.cpp
                  src/RDSGroupDecoder.cpp
                  src/RDSProcess.cpp
                  src/ChannelSettings.cpp
                  src/Utils.cpp)

set(BASIC_HEADERS src/ChannelSettings.h
                  src/Definitions.h
                  src/DownConvert.h
                  src/Filter.h
                  src/FirFilter.h
                  src/FmDecode.h
                  src/FreqShift.h
                  src/IirFilter.h
                  src/RadioReceiver.h
                  src/RDSGroupDecoder.h
                  src/RDSProcess.h
                  src/RTL_SDR_Source.h
                  src/Utils.h
                  src/XMLUtils.h
                  src/filtercoef.h)

set(DEPLIBS ${TINYXML_LIBRARIES}
            ${RTLSDR_LIBRARIES}
            ${LIBUSB_1_LIBRARIES})

addon_version(pvr.rtl.radiofm RTL_RADIOFM)
add_definitions(-DRTL_RADIOFM_VERSION=${RTL_RADIOFM_VERSION})

build_addon(pvr.rtl.radiofm BASIC DEPLIBS)

include(CPack)
