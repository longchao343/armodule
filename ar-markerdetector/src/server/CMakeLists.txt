cmake_minimum_required (VERSION 2.8)

#find_path (ALVAR_SDK
#	NAMES include/Alvar.h
#	HINTS 
#	"${CMAKE_CURRENT_SOURCE_DIR}/../alvar-2.0.0-sdk-linux64-gcc44/" 
#	"${CMAKE_CURRENT_SOURCE_DIR}/../alvar-2.0.0-sdk-linux64-gcc44/" 
#	"${CMAKE_CURRENT_SOURCE_DIR}/../../alvar-2.0.0-sdk-linux64-gcc44/" 
#	"${CMAKE_CURRENT_SOURCE_DIR}/../../../alvar-2.0.0-sdk-linux64-gcc44/")
#set (ALVAR_INC "${ALVAR_SDK}/include" CACHE PATH "ALVAR SDK include dir")
#find_library(ALVAR_LIB alvar200 HINTS ${ALVAR_SDK}/bin)
#include_directories(${ALVAR_INC})

# TODO: Why install does not work?
#install(FILES ${ALVAR_LIB} DESTINATION ${CMAKE_INSTALL_LIBDIR})

message ("Found library AlvarSdk from:" ${ALVAR_SDK})
message ("Found library AlvarInc from:" ${ALVAR_INC})
find_library(ALVAR_LIB alvar200 ${ALVAR_SDK}/bin)
message(STATUS "Remember to copy alvar library to global lib-dir: ${ALVAR_LIB}") 

#find_library(IRRLICHT_LIB libIrrlicht HINTS ${ALVAR_SDK}/bin)

# SOUP
#include_directories(${SOUP_INCLUDE_DIRS})

#IRRLICHT
#set (IRRLICHT_LIBPATH "/home/$ENV{USER}/nubomedia/irrlicht-code/lib/Linux")

set (IRRLICHT_LIBPATH "/usr/local/lib")
find_library(IRRLICHT_LIB Irrlicht ${IRRLICHT_LIBPATH})

#set (IRRLICHT_LIBPATH "/usr/local/lib")
#set (IRRLICHT_LIBPATH "/home/$ENV{USER}/nubomedia/irrlicht-code/lib/Linux")
#set (IRRLICHT_LIBPATH "/home/$ENV{USER}/nubomedia/irrlicht-code/lib/Linux")
#find_library(IRRLICHT_LIB Irrlicht ${IRRLICHT_LIBPATH})
#link_directories(${IRRLICHT_LIBPATH})

# Generate code
include (CodeGenerator)

# Possible parameters
# set (MULTI_VALUE_PARAMS
#   MODELS
#   INTERFACE_LIB_EXTRA_SOURCES
#   INTERFACE_LIB_EXTRA_HEADERS
#   INTERFACE_LIB_EXTRA_INCLUDE_DIRS
#   INTERFACE_LIB_EXTRA_LIBRARIES
#   SERVER_IMPL_LIB_EXTRA_SOURCES
#   SERVER_IMPL_LIB_EXTRA_HEADERS
#   SERVER_IMPL_LIB_EXTRA_INCLUDE_DIRS
#   SERVER_IMPL_LIB_EXTRA_LIBRARIES
#   MODULE_EXTRA_INCLUDE_DIRS
#   MODULE_EXTRA_LIBRARIES
#   SERVER_IMPL_LIB_FIND_CMAKE_EXTRA_LIBRARIES
# )


	generate_code (
	  MODELS ${CMAKE_CURRENT_SOURCE_DIR}/interface
	  INTERFACE_LIB_EXTRA_INCLUDE_DIRS ${ALVAR_INC}
	  SERVER_IMPL_LIB_EXTRA_SOURCES implementation/objects/Process.cpp
	  SERVER_IMPL_LIB_EXTRA_HEADERS implementation/objects/Process.h
	  SERVER_IMPL_LIB_EXTRA_INCLUDE_DIRS ${ALVAR_INC} ${SOUP_INCLUDE_DIRS} ${IRRLICHT_INC}
	  SERVER_IMPL_LIB_EXTRA_LIBRARIES ${ALVAR_LIB} ${SOUP_LIBRARIES} ${IRRLICHT_LIB}
	  SERVER_STUB_DESTINATION ${CMAKE_CURRENT_SOURCE_DIR}/implementation/objects
	)

