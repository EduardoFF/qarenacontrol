#INCLUDE(FindPackageHandleStandardArgs)
#INCLUDE(HandleLibraryTypes)

SET(LCM_IncludeSearchPaths
  usr/include/
  usr/local/include/
  opt/local/include
)

SET(LCM_LibrarySearchPaths
  usr/lib/
  usr/local/lib/
  opt/local/lib/
)

#FIND_PATH(LCM_INCLUDE_DIR lcm/lcm.h
  #PATHS ${LCM_IncludeSearchPaths}
#)
FIND_LIBRARY(LCM_LIBRARY
  NAMES lcm
  PATHS ${LCM_LibrarySearchPaths}
)

MESSAGE(STATUS "LCM Find Result: ${LCM_LIBRARY}")

IF (LCM_LIBRARY)
  SET(LCM_LIBRARY_FOUND TRUE)
ENDIF (LCM_LIBRARY)


IF (LCM_LIBRARY_FOUND)
  IF (NOT LCM_FIND_QUIETLY)
    MESSAGE(STATUS "Found LCM Library: ${LCM_LIBRARY}")
  ENDIF (NOT LCM_FIND_QUIETLY)
ELSE (LCM_LIBRARY_FOUND)
  IF (LCM_FIND_REQUIRED)
    MESSAGE(FATAL_ERROR "Could not find LCM")
  ENDIF (LCM_FIND_REQUIRED)
ENDIF (LCM_LIBRARY_FOUND)


