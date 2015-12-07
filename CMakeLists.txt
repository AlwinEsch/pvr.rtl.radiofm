project(pvr.rtl.radiofm)

cmake_minimum_required(VERSION 2.6)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR} ${CMAKE_SOURCE_DIR}/lib/rtl-sdr/cmake/Modules)

enable_language(CXX)

find_package(TinyXML REQUIRED)
find_package(kodi REQUIRED)
find_package(kodiplatform REQUIRED)
find_package(platform REQUIRED)

if (NOT WIN32)
  add_definitions(-DUSE_DEMUX -fPIC -g -O2)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
endif()

include_directories(${kodiplatform_INCLUDE_DIRS}
                    ${platform_INCLUDE_DIRS}
                    ${KODI_INCLUDE_DIR}
                    ${TINYXML_INCLUDE_DIRS}
                    ${PROJECT_SOURCE_DIR}/lib/rtl-sdr/include)

add_subdirectory(lib/rtl-sdr)

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

set(DEPLIBS ${kodiplatform_LIBRARIES}
            ${platform_LIBRARIES}
            ${TINYXML_LIBRARIES}
            rtlsdr_static)

build_addon(pvr.rtl.radiofm BASIC DEPLIBS)

include(CPack)
