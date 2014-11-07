# Try to find GALIB
# See http://lancet.mit.edu/ga/
#
# Once run this will define: 
# 
# GALIB_FOUND        = system has GALIB lib
#
# GALIB_LIBRARIES    = full path to the libraries
# GALIB_INCLUDE_DIR  = where to find headers 


#
# Find the library
#
FIND_LIBRARY(BOOST_LIBRARY
  NAMES
  boost_thread
  PATHS
  /usr/lib/x86_64-linux-gnu/
  DOC "Boost thread location"
)

INCLUDE(FindPackageHandleStandardArgs)
FIND_PACKAGE_HANDLE_STANDARD_ARGS( BOOSTLIB DEFAULT_MSG
	BOOST_LIBRARY)

IF(BOOST_LIBRARY)	
	SET(BOOST_LIBRARIES	  ${BOOST_LIBRARY})
ENDIF(BOOST_LIBRARY)

MARK_AS_ADVANCED(BOOST_LIBRARY)
