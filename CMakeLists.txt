cmake_minimum_required (VERSION 2.6)
SET(CMAKE_COLOR_MAKEFILE ON)

project (QArenaControl)

SET( PROJECT_SOURCE_ROOT ${CMAKE_CURRENT_SOURCE_DIR} )
SET( PROJECT_BINARY_ROOT ${CMAKE_CURRENT_BINARY_DIR} )
SET( VERSION_MAJOR 0 )
SET( VERSION_MINOR 1 )
SET( VERSION_PATCH 1 )
SET( VERSION_STRING "${VERSION_MAJOR}.${VERSION_MINOR}.${VERSION_PATCH}" )

OPTION( BUILD_EXAMPLES "If on, build executable examples." ON )

# Put executables in bin/
SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/bin")

# Include our own directories.
INCLUDE_DIRECTORIES( "${CMAKE_CURRENT_SOURCE_DIR}/include" )

# Need to find boost at build time only if we are actually compiling examples.
FIND_PACKAGE( Boost COMPONENTS program_options system thread REQUIRED)
INCLUDE_DIRECTORIES( ${Boost_INCLUDE_DIRS} )

# Try to find Doxygen
FIND_PROGRAM( DOXYGEN_CMD doxygen )

# Descend into subdirectories
ADD_SUBDIRECTORY( include )
IF( DOXYGEN_CMD )
   ADD_SUBDIRECTORY( doc )
ENDIF()
ADD_SUBDIRECTORY( src )

