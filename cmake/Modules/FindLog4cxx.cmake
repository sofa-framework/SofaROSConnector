
INCLUDE(FindPackageHandleStandardArgs)

if (WIN32)
  set(X86_ENV "PROGRAMFILES(X86)")
  set(ENV_PROGRAM_FILES_X86 $ENV{${X86_ENV}})

  FILE(TO_CMAKE_PATH ${ENV_PROGRAM_FILES_X86} PROGRAM_FILES_X86)

  SET(SEARCH_PATHS
      $ENV{ProgramFiles}/log4cxx/include
      $ENV{SystemDrive}/log4cxx/include
      ${PROGRAM_FILES_X86}/log4cxx/include
      C:/SDK/win_ros/compile_deps/x64/include
  )

  SET(SEARCH_PATHS_LIB
      $ENV{ProgramFiles}/log4cxx/lib
      $ENV{SystemDrive}/log4cxx/lib
      ${PROGRAM_FILES_X86}/log4cxx/lib
      C:/SDK/win_ros/compile_deps/x64/lib
  )


  FIND_PATH(LOG4CXX_INCLUDE_DIRS
            PATHS ${SEARCH_PATHS}
            NAMES logger.h
            PATH_SUFFIXES log4cxx)

  FIND_LIBRARY(LOG4CXX_LIBRARY_RELEASE
    NAMES log4cxx
    PATHS ${SEARCH_PATHS_LIB}
    )

  FIND_LIBRARY(LOG4CXX_LIBRARY_DEBUG
    NAMES log4cxx_d
    PATHS ${SEARCH_PATHS_LIB}
    )

  SET(LOG4CXX_LIBRARIES
    debug ${LOG4CXX_LIBRARY_DEBUG}
    optimized ${LOG4CXX_LIBRARY_RELEASE}
    )

  # post-process inlude path
  IF(LOG4CXX_INCLUDE_DIRS)
      STRING(REGEX REPLACE log4cxx$$ "" LOG4CXX_INCLUDE_DIRS ${LOG4CXX_INCLUDE_DIRS})
      SET(LOG4CXX_INCLUDE_DIRS ${LOG4CXX_INCLUDE_DIRS} CACHE PATH "log4cxx include dirs" FORCE)
  ENDIF()

  FIND_PACKAGE_HANDLE_STANDARD_ARGS(Log4cxx DEFAULT_MSG LOG4CXX_INCLUDE_DIRS LOG4CXX_LIBRARIES)

  # only visible in advanced view
  MARK_AS_ADVANCED(LOG4CXX_INCLUDE_DIRS LOG4CXX_LIBRARIES)

else()

  FIND_PATH(LOG4CXX_INCLUDE_DIRS logger.h PATHS /include/log4cxx /usr/include/log4cxx /usr/local/include/log4cxx /opt/local/include/log4cxx )
  FIND_LIBRARY(LOG4CXX_LIBRARIES NAMES log4cxx log4cxxd PATHS /lib /usr/lib /usr/local/lib /opt/local/lib )

  IF(LOG4CXX_INCLUDE_DIRS AND LOG4CXX_LIBRARIES)
    SET(LOG4CXX_FOUND 1)
    #remove last /log4cxx string
    STRING(REGEX REPLACE "/log4cxx" "" LOG4CXX_INCLUDE_DIR_SUP_LEVEL ${LOG4CXX_INCLUDE_DIRS})
    SET (LOG4CXX_INCLUDE_DIR ${LOG4CXX_INCLUDE_DIR_SUP_LEVEL} ${LOG4CXX_INCLUDE_DIRS} )
    if(NOT LOG4CXX_FIND_QUIETLY)
     message(STATUS "Found log4cxx: ${LOG4CXX_LIBRARIES}")
    endif(NOT LOG4CXX_FIND_QUIETLY)
  ELSE(LOG4CXX_INCLUDE_DIRS AND LOG4CXX_LIBRARIES)
    SET(LOG4CXX_FOUND 0 CACHE BOOL "Not found log4cxx library")
    message(STATUS "NOT Found log4cxx, disabling it")
  ENDIF(LOG4CXX_INCLUDE_DIRS AND LOG4CXX_LIBRARIES)

  MARK_AS_ADVANCED(LOG4CXX_INCLUDE_DIRS LOG4CXX_LIBRARIES)

endif()
