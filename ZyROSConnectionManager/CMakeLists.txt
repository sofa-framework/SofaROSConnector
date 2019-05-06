project(ZyROSConnectionManager)

message(STATUS "Generating ROS message and service type header files.")

# TODO: Specify ROS_INSTALL_PREFIX and ROS_DISTRIBUTION arguments based on environment variables
execute_process(COMMAND "${CMAKE_CURRENT_SOURCE_DIR}/list_ros_message_types.sh" "${CMAKE_CURRENT_BINARY_DIR}")

if (EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/ZyROS_MessageType_Instantiations.h" AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/ZyROS_ServiceType_Instantiations.h")
    message(STATUS "Header files generated successfully.")
else()
    message(FATAL "Header file generation failed!")
endif()

set(HEADER_FILES
    initZyROSConnectionManager.h
    ZyROSConnectionManager.h
)

set(SOURCE_FILES
    initZyROSConnectionManager.cpp
    ZyROSConnectionManager.cpp
)

find_package(ROS QUIET REQUIRED)
message(STATUS "ROS found: ${ROS_FOUND}")
find_package(Boost COMPONENTS regex thread signals QUIET REQUIRED)
find_package(Log4cxx QUIET REQUIRED)

# Boost
find_package(Boost COMPONENTS system chrono thread REQUIRED QUIET)
include_directories(${Boost_INCLUDE_DIR})
link_directories(${Boost_LIBRARY_DIRS})

# SOFA include directories
include_directories(${CMAKE_CURRENT_SOURCE_DIR})
include_directories(${CMAKE_CURRENT_BINARY_DIR})
include_directories(${CMAKE_SOURCE_DIR})
include_directories(${CMAKE_SOURCE_DIR}/modules)

# Dependencies
include_directories(${CMAKE_SOURCE_DIR}/applications/plugins/SofaROSConnector/ZyROSConnector)
include_directories(${CMAKE_SOURCE_DIR}/applications/plugins/SofaROSConnector)
include_directories(${CMAKE_SOURCE_DIR}/SofaKernel/framework)
include_directories(${CMAKE_BINARY_DIR}/include)

# ROS (hydro)
include_directories(${ROS_INCLUDE_DIR})
include_directories(${LOG4CXX_INCLUDE_DIRS})

# Build list of available ROS message types

SET(ROS_DISTRO "$ENV{ROS_DISTRO}")
if ("${ROS_DISTRO}" STREQUAL "")
    SET(ROS_DISTRO "kinetic")
endif()

SET(ROS_INSTALL_PREFIX "$ENV{ROS_INSTALL_PREFIX}")
if ("${ROS_INSTALL_PREFIX}" STREQUAL "")
    SET(ROS_INSTALL_PREFIX "/opt/ros/kinetic")
endif()

execute_process(COMMAND "${CMAKE_CURRENT_SOURCE_DIR}/list_ros_message_types.sh" "${ROS_DISTRO}" "${ROS_INSTALL_PREFIX}" OUTPUT_VARIABLE ROS_MSG_TYPES OUTPUT_STRIP_TRAILING_WHITESPACE)
message(STATUS "List of ROS message types: ${ROS_MSG_TYPES}")
string(REPLACE "\n" ";" ROS_MSG_TYPES_LIST ${ROS_MSG_TYPES})
foreach(ROS_MSG_TYPE ${ROS_MSG_TYPES_LIST})
    message(STATUS "Message type: ${ROS_MSG_TYPE}")
endforeach()

set(BINDING_HEADER_FILES     ${CMAKE_CURRENT_BINARY_DIR}/ZyROS_MessageType_Instantiations.h
                             ${CMAKE_CURRENT_BINARY_DIR}/ZyROS_ServiceType_Instantiations.h)

set(BINDING_SOURCE_FILES     ${CMAKE_CURRENT_BINARY_DIR}/ZyROS_MessageTypes_Instantiations.cpp
                             ${CMAKE_CURRENT_BINARY_DIR}/ZyROS_ServiceTypes_Instantiations.cpp)

add_definitions("-DSOFA_BUILD_ZY_ROS_CONNECTION_MANAGER")
add_definitions(-DROS_BUILD_SHARED_LIBS)

add_library(ZyROSConnectorBindings ${BINDING_HEADER_FILES} ${BINDING_SOURCE_FILES})
target_compile_options(ZyROSConnectorBindings PRIVATE "-fpermissive")

target_link_libraries(ZyROSConnectorBindings ${ROS_LIBRARY} ${ROS_CPP_LIBRARY} ${ROS_CPPCOMMON_LIBRARY} ${ROS_TIME_LIBRARY} ${ROS_CONSOLE_LIBRARY} ${ROS_CPP_SERIALIZATION_LIBRARY})

target_link_libraries(ZyROSConnectorBindings ${Boost_REGEX_LIBRARY} ${Boost_THREAD_LIBRARY} ${Boost_SIGNALS_LIBRARY})
target_link_libraries(ZyROSConnectorBindings ${LOG4CXX_LIBRARIES})

add_library(${PROJECT_NAME} SHARED ${HEADER_FILES} ${SOURCE_FILES})

set_target_properties(${PROJECT_NAME} PROPERTIES FOLDER "Plugins")
set_target_properties(${PROJECT_NAME} PROPERTIES DEBUG_POSTFIX "_d")

target_link_libraries(${PROJECT_NAME} SofaCore SofaHelper SofaDefaultType SofaBaseCollision SofaConstraint SofaSimulationCommon SofaMiscFem tinyxml newmat)
target_link_libraries(${PROJECT_NAME} ZyROSConnector ZyROSConnectorBindings)
target_link_libraries(${PROJECT_NAME} ${ROS_LIBRARY} ${ROS_CPP_LIBRARY} ${ROS_CPPCOMMON_LIBRARY} ${ROS_TIME_LIBRARY} ${ROS_CONSOLE_LIBRARY} ${ROS_CPP_SERIALIZATION_LIBRARY})

target_link_libraries(${PROJECT_NAME} ${Boost_REGEX_LIBRARY} ${Boost_THREAD_LIBRARY} ${Boost_SIGNALS_LIBRARY})
target_link_libraries(${PROJECT_NAME} ${LOG4CXX_LIBRARIES})

find_package(SofaTest QUIET)
if(SofaTest_FOUND)
    add_subdirectory(ZyROSConnector_test)
endif()

sofa_create_package(ZyROSConnectorBindings "0.0.1" ZyROSConnectorBindings ZyROSConnectorBindings)
sofa_create_package(${PROJECT_NAME} "0.0.1" ${PROJECT_NAME} ${PROJECT_NAME})
