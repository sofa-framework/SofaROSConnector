project(ZySimulationAnalyzer)

find_package(ROS REQUIRED)
message(STATUS "ROS found: ${ROS_FOUND}")

set(HEADER_FILES
    initZyROSSimulationAnalyzer.h
    ZyROSSimulationAnalyzer.h
)

set(SOURCE_FILES
    initZyROSSimulationAnalyzer.cpp
    ZyROSSimulationAnalyzer.cpp
)

include_directories(${CMAKE_SOURCE_DIR}/applications/plugins/SofaROSConnector/ZyROSConnector)
include_directories(${CMAKE_SOURCE_DIR}/applications/plugins/SofaROSConnector/ZyROSPublishingHandler)

# SOFA include directories
include_directories(${CMAKE_CURRENT_SOURCE_DIR})
include_directories(${CMAKE_SOURCE_DIR})
include_directories(${CMAKE_SOURCE_DIR}/modules)

# ROS (hydro)
include_directories(${ROS_INCLUDE_DIR})
include_directories(${LOG4CXX_INCLUDE_DIRS})

add_definitions("-DSOFA_BUILD_ZY_SIMULATION_ANALYSIS")

add_library(${PROJECT_NAME} SHARED ${HEADER_FILES} ${SOURCE_FILES})

set_target_properties(${PROJECT_NAME} PROPERTIES FOLDER "Plugins")

target_link_libraries(${PROJECT_NAME} SofaCore SofaHelper SofaDefaultType SofaBaseCollision SofaConstraint SofaSimulationCommon SofaMiscFem tinyxml newmat)

if (ROS_FOUND)
        target_link_libraries(${PROJECT_NAME} ZyROSConnector)
        target_link_libraries(${PROJECT_NAME} ZyROSPublishingHandler)
endif()
target_link_libraries(${PROJECT_NAME} ${ROS_LIBRARY} ${ROS_CPP_LIBRARY} ${ROS_CPPCOMMON_LIBRARY} ${ROS_TIME_LIBRARY} ${ROS_CONSOLE_LIBRARY} ${ROS_CPP_SERIALIZATION_LIBRARY})

target_link_libraries(${PROJECT_NAME} ${Boost_REGEX_LIBRARY} ${Boost_THREAD_LIBRARY} ${Boost_SIGNALS_LIBRARY})
target_link_libraries(${PROJECT_NAME} ${LOG4CXX_LIBRARIES})

sofa_create_package(${PROJECT_NAME} "0.0.1" ${PROJECT_NAME} ${PROJECT_NAME})
