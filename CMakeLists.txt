project(pvr.rtl.radiofm)

cmake_minimum_required(VERSION 2.6)

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR})

enable_language(CXX)

find_package(xbmc REQUIRED)
find_package(xbmcplatform REQUIRED)
find_package(RtlSdr REQUIRED)

add_definitions(-DUSE_DEMUX=1 -fPIC -g -O2 -std=gnu++11)
include_directories(${xbmcplatform_INCLUDE_DIRS}
                    ${XBMC_INCLUDE_DIR})

set(BASIC_SOURCES src/client.cpp
                  src/RadioReceiver.cpp
                  src/RTL_SDR_Source.cpp
                  src/FirFilter.cpp
                  src/IirFilter.cpp
                  src/FmDecode.cpp
                  src/DownConvert.cpp
                  src/FreqShift.cpp
                  src/RDSGroupDecoder.cpp
                  src/RDSProcess.cpp
                  src/ChannelSettings.cpp)

set(DEPLIBS ${xbmcplatform_LIBRARIES}
            ${RTL_SDR_LIBRARIES})

build_addon(pvr.rtl.radiofm BASIC DEPLIBS)

include(CPack)
