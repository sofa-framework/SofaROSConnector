project(ZyROSKinematics)

set(HEADER_FILES
    initZyRosKinematics.h
    ZyRosKinematics.h
)

set(SOURCE_FILES
    initZyRosKinematics.cpp
    ZyRosKinematics.cpp
)


# Boost
find_package(Boost COMPONENTS system chrono thread REQUIRED QUIET)
include_directories(${Boost_INCLUDE_DIR})
link_directories(${Boost_LIBRARY_DIRS})

# SOFA include directories
include_directories(${CMAKE_CURRENT_SOURCE_DIR})
include_directories(${CMAKE_SOURCE_DIR})
include_directories(${CMAKE_SOURCE_DIR}/modules)

find_package(ROS QUIET REQUIRED)
message(STATUS "ROS found: ${ROS_FOUND}")

find_package(Log4cxx QUIET REQUIRED)
include_directories(${ROS_INCLUDE_DIR})
include_directories(${LOG4CXX_INCLUDE_DIRS})

include_directories(${CMAKE_SOURCE_DIR}/applications/plugins/SofaROSConnector/ZySOFAControllers)
include_directories(${CMAKE_SOURCE_DIR}/applications/plugins/SofaROSConnector/ZyROSConnector)
include_directories(${CMAKE_SOURCE_DIR}/applications/plugins/SofaROSConnector/ZyROSConnectionManager)
include_directories(${CMAKE_SOURCE_DIR}/applications/plugins/SofaROSConnector/ZyROSPublishingHandler)
include_directories(${CMAKE_SOURCE_DIR}/applications/plugins/SofaROSConnector/ZyVelocityApproximationHandler)

add_definitions("-DSOFA_BUILD_ZYROSKINEMATICS")

add_library(${PROJECT_NAME} SHARED ${HEADER_FILES} ${SOURCE_FILES})
set_target_properties(${PROJECT_NAME} PROPERTIES FOLDER "Plugins")
set_target_properties(${PROJECT_NAME} PROPERTIES DEBUG_POSTFIX "_d")

target_link_libraries(${PROJECT_NAME} SofaCore SofaHelper SofaDefaultType SofaBaseCollision SofaConstraint SofaSimulationCommon)
if (ROS_FOUND)
        target_link_libraries(${PROJECT_NAME} ZyROSConnector)
        target_link_libraries(${PROJECT_NAME} ZyROSPublishingHandler)
        target_link_libraries(${PROJECT_NAME} ZyVelocityApproximationHandler)
endif()
target_link_libraries(${PROJECT_NAME} ${ROS_LIBRARY} ${ROS_CPP_LIBRARY} ${ROS_CPPCOMMON_LIBRARY} ${ROS_TIME_LIBRARY} ${ROS_CONSOLE_LIBRARY} ${ROS_CPP_SERIALIZATION_LIBRARY})

target_link_libraries(${PROJECT_NAME} ${Boost_REGEX_LIBRARY} ${Boost_THREAD_LIBRARY} ${Boost_SIGNALS_LIBRARY})
