cmake_minimum_required(VERSION 2.8)

if (EXISTS "lib/libalvar200.so")
	file(COPY lib/libalvar200.so DESTINATION debian/tmp)
endif()
if (EXISTS "lib/libIrrlicht.so.1.9.0")
	file(COPY lib/libIrrlicht.so.1.9.0 DESTINATION debian/tmp)
endif()

project ("ar-markerdetector")

message ("If KurentoHelpers is not found, then it is necessary to install
          kms-cmake-utils package from the kurento repository")

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/CMake")
find_package(KurentoHelpers REQUIRED)

include (KurentoGitHelpers)
message ("If CodeGenerator is not found, then it is necessary to install
          kms-core package from the kurento repository")
include (CodeGenerator)

get_values_from_model(PREFIX VALUE MODELS ${CMAKE_SOURCE_DIR}/src/server/interface KEYS version)

get_git_version (PROJECT_VERSION ${VALUE_VERSION})

string(REPLACE "." ";" VERSION_LIST ${PROJECT_VERSION})
list(GET VERSION_LIST 0 PROJECT_VERSION_MAJOR)
list (LENGTH VERSION_LIST _len)
if (${_len} GREATER 1)
  list(GET VERSION_LIST 1 PROJECT_VERSION_MINOR)
endif ()
if (${_len} GREATER 2)
  list(GET VERSION_LIST 2 PROJECT_VERSION_PATCH)
endif ()

find_package(PkgConfig)

#set (GST_REQUIRED 1.3.3)
set (GST_REQUIRED 1.5.0)
set (GLIB_REQUIRED 2.38)
set (OPENCV_REQUIRED 2.0.0)
set (SOUP_REQUIRED 2.40) # For URL support

#set (IRRLICHT_REQUIRED 1.8)

#gst-plugins dependencies
pkg_check_modules(GSTREAMER REQUIRED gstreamer-1.5>=${GST_REQUIRED})
pkg_check_modules(GSTREAMER_BASE REQUIRED gstreamer-base-1.5>=${GST_REQUIRED})
pkg_check_modules(GSTREAMER_VIDEO REQUIRED gstreamer-video-1.5>=${GST_REQUIRED})
pkg_check_modules(GSTREAMER_CHECK REQUIRED gstreamer-check-1.5>=${GST_REQUIRED})
#pkg_check_modules(GSTREAMER REQUIRED gstreamer-1.0>=${GST_REQUIRED})
#pkg_check_modules(GSTREAMER_BASE REQUIRED gstreamer-base-1.0>=${GST_REQUIRED})
#pkg_check_modules(GSTREAMER_VIDEO REQUIRED gstreamer-video-1.0>=${GST_REQUIRED})
#pkg_check_modules(GSTREAMER_CHECK REQUIRED gstreamer-check-1.0>=${GST_REQUIRED})
pkg_check_modules(KMSCORE REQUIRED kmscore)
pkg_check_modules(OPENCV REQUIRED opencv>=${OPENCV_REQUIRED})
pkg_check_modules(SOUP REQUIRED libsoup-2.4>=${SOUP_REQUIRED})

set (VERSION ${PROJECT_VERSION})
set (PACKAGE ${PROJECT_NAME})
set (GETTEXT_PACKAGE "ar-markerdetector")

include(GNUInstallDirs)

set (CMAKE_INSTALL_GST_PLUGINS_DIR ${CMAKE_INSTALL_LIBDIR}/gstreamer-1.0)
set (KURENTO_MODULES_SO_DIR ${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_LIBDIR}/${KURENTO_MODULES_DIR_INSTALL_PREFIX})



configure_file(${CMAKE_CURRENT_SOURCE_DIR}/config.h.cmake ${CMAKE_CURRENT_BINARY_DIR}/config.h)

#set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -g -DHAVE_CONFIG_H -Werror -Wall")
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -DHAVE_CONFIG_H -Wall -Werror -std=c++11")
#set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -g -DHAVE_CONFIG_H -Werror")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -g -DHAVE_CONFIG_H -Werror")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -DHAVE_CONFIG_H -Werror -std=c++11")
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -DHAVE_CONFIG_H -Werror -std=c++11")

include_directories(${CMAKE_CURRENT_BINARY_DIR})
include_directories(${CMAKE_CURRENT_SOURCE_DIR})
set (ALVAR_SDK "/home/$ENV{USER}/nubomedia/alvar-2.0.0-sdk-linux64-gcc44")
set (ALVAR_INC ${ALVAR_SDK}/include)


#set (IRRLICHT_LIBPATH "/home/$ENV{USER}/nubomedia/irrlicht-code/lib/Linux")
#set (IRRLICHT_INC "/home/$ENV{USER}/nubomedia/irrlicht-code/include")
#set (IRRLICHT_LIBPATH "/usr/local/lib")
set (IRRLICHT_INC "/usr/local/include")


#set (IRRLICHT_LIBPATH "/usr/local/lib")
#find_library(IRRLICHT_LIB Irrlicht ${IRRLICHT_LIBPATH})

#NOW BEGIN
#include_directories(${IRRLICHT_INC})
#find_library(IRRLICHT_LIB Irrlicht ${IRRLICHT_LIBPATH})
#add_library(${IRRLICHT_LIB} STATIC )
#link_directories(${IRRLICHT_LIBPATH})
#macri(ASSERT test comment
#assert (${IRRLICHT_LIB} "Bizarre - Unable to find library Irrlicht") 
#if(NOT EXISTS ${IRRLICHT_LIB})
#  message ("Unable to find library Irrlicht:" ${IRRLICHT_LIB})
#  message ("Irrlicht library path:" ${IRRLICHT_LIBPATH})
#endif (NOT ${IRRLICHT_LIB})


#message("Irrlich include : ${IRRLICHT_INC}")
#NOW END

message(STATUS "Compiling : ${PROJECT_NAME}-${PROJECT_VERSION}")

add_subdirectory (src)

message(KURENTO_MODULES_SO_DIR)
message(${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_LIBDIR}/${KURENTO_MODULES_DIR_INSTALL_PREFIX})
