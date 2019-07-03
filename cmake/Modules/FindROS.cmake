# -*- cmake -*- 
#######################################################################
# Software License Agreement (BSD License)                            #
#                                                                     #
#  Copyright (c) 2011, MBARI.                                         #
#  All rights reserved.                                               #
#                                                                     #
#  Redistribution and use in source and binary forms, with or without #
#  modification, are permitted provided that the following conditions #
#  are met:                                                           #
#                                                                     #
#   * Redistributions of source code must retain the above copyright  #
#     notice, this list of conditions and the following disclaimer.   #
#   * Redistributions in binary form must reproduce the above         #
#     copyright notice, this list of conditions and the following     #
#     disclaimer in the documentation and/or other materials provided #
#     with the distribution.                                          #
#   * Neither the name of the TREX Project nor the names of its       #
#     contributors may be used to endorse or promote products derived #
#     from this software without specific prior written permission.   #
#                                                                     #
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS #
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT   #
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS   #
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE      #
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, #
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,#
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;    #
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER    #
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT  #
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN   #
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE     #
# POSSIBILITY OF SUCH DAMAGE.                                         #
#######################################################################

if (UNIX)
    if("${ROS_ROOT}" STREQUAL "")
        message(WARNING "ROS_ROOT is not set!")
        if("$ENV{ROS_ROOT}" STREQUAL "")
            message(FATAL_ERROR "ROS_ROOT environment variable is not set!")
        else()
            string(REPLACE "/share/ros" "" REAL_ROS_ROOT $ENV{ROS_ROOT})
            message(STATUS "REAL_ROS_ROOT: ${REAL_ROS_ROOT}")
            set(ROS_ROOT ${REAL_ROS_ROOT} CACHE PATH "Root path for ROS" FORCE)
        endif()
    else()
        string(REPLACE "/share/ros" "" REAL_ROS_ROOT ${ROS_ROOT})
        message(STATUS "REAL_ROS_ROOT: ${REAL_ROS_ROOT}")
        set(ROS_ROOT ${REAL_ROS_ROOT} CACHE PATH "Root path for ROS" FORCE)
    endif()
else()
    if("$ENV{ROS_ROOT}" STREQUAL "")
        message(WARNING "ROS_ROOT environment variable is not set!")
    else()
        message(STATUS "ROS_ROOT from environment variable: ${ROS_ROOT}")
        set(ROS_ROOT $ENV{ROS_ROOT} CACHE PATH "Root path for ROS" FORCE)
    endif()
endif()

message(STATUS "ROS_ROOT: ${ROS_ROOT}")

set(SEARCH_PATHS_INCLUDE ${ROS_ROOT}/include
    C:/opt/ros/hydro/x86/include)

set(SEARCH_PATHS_LIB ${ROS_ROOT}/lib
    C:/opt/ros/hydro/x86/lib)

if (WIN32)
    find_path(ROS_INCLUDE_DIR ros/ros.h
        HINTS ${SEARCH_PATHS_INCLUDE}
        )

    message(STATUS "ROS_INCLUDE_DIR: ${ROS_INCLUDE_DIR}")

    find_library(ROS_LIBRARY NAMES roslib
        HINTS ${SEARCH_PATHS_LIB})

    find_library(ROS_CPP_LIBRARY NAMES roscpp
        HINTS ${SEARCH_PATHS_LIB})

    find_library(ROS_CPPCOMMON_LIBRARY NAMES cpp_common
        HINTS ${SEARCH_PATHS_LIB})

    find_library(ROS_TIME_LIBRARY NAMES rostime
        HINTS ${SEARCH_PATHS_LIB})

    find_library(ROS_CONSOLE_LIBRARY NAMES rosconsole
        HINTS ${SEARCH_PATHS_LIB})

    find_library(ROS_CPP_SERIALIZATION_LIBRARY NAMES roscpp_serialization
        HINTS ${SEARCH_PATHS_LIB})

    message(STATUS "ROS libraries: roslib -- ${ROS_LIBRARY}; roscpp -- ${ROS_CPP_LIBRARY}; cpp_common -- ${ROS_CPPCOMMON_LIBRARY}; rostime -- ${ROS_TIME_LIBRARY}; rosconsole -- ${ROS_CONSOLE_LIBRARY}")

    set(ROS_LIBRARIES "${ROS_LIBRARY};${ROS_CPP_LIBRARY};${ROS_CPPCOMMON_LIBRARY};${ROS_TIME_LIBRARY};${ROS_CONSOLE_LIBRARY};${ROS_CPP_SERIALIZATION_LIBRARY}")
    set(ROS_INCLUDE_DIRS ${ROS_INCLUDE_DIR} )

    include(FindPackageHandleStandardArgs)
    # handle the QUIETLY and REQUIRED arguments and set LIBXML2_FOUND to TRUE
    # if all listed variables are TRUE
    find_package_handle_standard_args(ROS  DEFAULT_MSG
        ROS_LIBRARY ROS_INCLUDE_DIR)

    mark_as_advanced(ROS_INCLUDE_DIR ROS_LIBRARY)

endif()

if (NOT WIN32)
    if(NOT OLD_ROS_ROOT EQUAL ROS_ROOT)
        message(STATUS "Looking for ROS configuration under: ${ROS_ROOT}")
        if ("$ENV{ROS_ROOT}" STREQUAL "")
            message(STATUS "ROS_ROOT not set in environment, cached value is ${ROS_ROOT}. Will set environment variable to cached value.")
            set(ENV{ROS_ROOT} "${ROS_ROOT}/share/ros")
        endif()
        find_file(ROS_CONFIG rosbuild.cmake HINTS ${ROS_ROOT}/share/ros/core/rosbuild)
        set(ROS_CONFIG ${ROS_CONFIG} CACHE FILE "ROS configuration file" FORCE)
        message(STATUS "Looking for ROS configuration: ${ROS_CONFIG}")
        set(OLD_ROS_ROOT ${ROS_ROOT} CACHE INTERNAL "Last ROS_ROOT value" FORCE)
    endif(NOT OLD_ROS_ROOT EQUAL ROS_ROOT)

    set(ROS_FOUND False)
    if(ROS_CONFIG)
        find_program(ROSPACK_EXE NAMES rospack DOC "rospack executable" PATHS ${ROS_ROOT}/bin)
        message(STATUS "rospack: ${ROSPACK_EXE}")
        include(${ROS_CONFIG})
        set(ROS_FOUND True)
        set(ROS_INCLUDE_DIR ${ROS_ROOT}/include)
        find_library(ROS_LIBRARY NAMES roslib
            HINTS ${ROS_ROOT}/lib)

        find_library(ROS_CPP_LIBRARY NAMES roscpp
            HINTS ${ROS_ROOT}/lib)

        find_library(ROS_CPPCOMMON_LIBRARY NAMES cpp_common
            HINTS ${ROS_ROOT}/lib)

        find_library(ROS_TIME_LIBRARY NAMES rostime
            HINTS ${ROS_ROOT}/lib)

        find_library(ROS_CONSOLE_LIBRARY NAMES rosconsole
            HINTS ${ROS_ROOT}/lib)

        find_library(ROS_CPP_SERIALIZATION_LIBRARY NAMES roscpp_serialization
            HINTS ${ROS_ROOT}/lib)

        message(STATUS "ROS libraries: roslib -- ${ROS_LIBRARY}; roscpp -- ${ROS_CPP_LIBRARY}; cpp_common -- ${ROS_CPPCOMMON_LIBRARY}; rostime -- ${ROS_TIME_LIBRARY}; rosconsole -- ${ROS_CONSOLE_LIBRARY}")

        set(ROS_LIBRARIES "${ROS_LIBRARY};${ROS_CPP_LIBRARY};${ROS_CPPCOMMON_LIBRARY};${ROS_TIME_LIBRARY};${ROS_CONSOLE_LIBRARY};${ROS_CPP_SERIALIZATION_LIBRARY}")
    endif(ROS_CONFIG)

    message(STATUS "ROS_CONFIG: ${ROS_CONFIG}")
    message(STATUS "ROS found: ${ROS_FOUND}")

    macro(FIND_ROS_PKG name)
        rosbuild_find_ros_package(${name})
        message(STATUS "Looking for ros package ${name}: ${${name}_PACKAGE_PATH}")
        set(ROS_${name}_PACKAGE_PATH ${${name}_PACKAGE_PATH} CACHE DIRECTORY
            "path for ros package ${name}" FORCE)
        if(${name}_PACKAGE_PATH)
            rosbuild_invoke_rospack(${name} ${name} INCLUDE_DIRS cflags-only-I)
            set(ROS_${name}_INCLUDE_DIRS ${${name}_INCLUDE_DIRS} CACHE STRING
                "${name} include paths" FORCE)
            mark_as_advanced(ROS_${name}_INCLUDE_DIRS)

            rosbuild_invoke_rospack(${name} ${name} CFLAGS cflags-only-other)
            set(ROS_${name}_CFLAGS ${${name}_CFLAGS} CACHE STRING
                "${name} compilation flags" FORCE)
            mark_as_advanced(ROS_${name}_CFLAGS)

            rosbuild_invoke_rospack(${name} ${name} LINK_PATH libs-only-L)
            set(ROS_${name}_LINK_PATH ${${name}_LINK_PATH} CACHE STRING
                "${name} link libs paths" FORCE)
            mark_as_advanced(ROS_${name}_LINK_PATH)

            rosbuild_invoke_rospack(${name} ${name} LINK_LIBS "libs-only-l")
            set(ROS_${name}_LINK_LIBS ${${name}_LINK_LIBS} CACHE STRING
                "${name} link libraries" FORCE)
            mark_as_advanced(ROS_${name}_LINK_LIBS)

            rosbuild_invoke_rospack(${name} ${name} LINK_FLAGS cflags-only-other)
            set(ROS_${name}_LINK_FLAGS ${${name}_LINK_FLAGS} CACHE STRING
                "${name} link flags" FORCE)
            mark_as_advanced(ROS_${name}_LINK_FLAGS)
        endif(${name}_PACKAGE_PATH)
    endmacro(FIND_ROS_PKG)
endif()
