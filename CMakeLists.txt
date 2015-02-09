project(pvr.rtl.radiofm)

cmake_minimum_required(VERSION 2.6)

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR})

enable_language(CXX)

find_package(kodi REQUIRED)
find_package(kodiplatform REQUIRED)
find_package(RtlSdr REQUIRED)

add_definitions(-DUSE_DEMUX -fPIC -g -O2 -std=gnu++11)
include_directories(${kodiplatform_INCLUDE_DIRS}
                    ${KODI_INCLUDE_DIR})

set(DEPLIBS ${kodiplatform_LIBRARIES}
            ${RTL_SDR_LIBRARIES})

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
                  src/ChannelSettings.cpp
                  src/utils.cpp)

build_addon(pvr.rtl.radiofm BASIC DEPLIBS)

include(CPack)
