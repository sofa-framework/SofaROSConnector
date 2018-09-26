project(ZySimulationAnalyzerHandler)

add_definitions(-DBUILD_SOFA_ZY_SIMULATION_ANALYSIS_HANDLER)

set(ZY_VELOCITYAPPROXIMATIONHANDLER_HEADER_FILES
        init_ZySimulationAnalyzerHandler.h
        ZySimulationAnalyzerHandler.h)

set(ZY_VELOCITYAPPROXIMATIONHANDLER_SOURCE_FILES
        ZySimulationAnalyzerHandler.cpp)

find_package(ROS REQUIRED)
message(STATUS "ROS found: ${ROS_FOUND}")

# ROS
include_directories(${ROS_INCLUDE_DIR})
include_directories(${LOG4CXX_INCLUDE_DIRS})

include_directories(${CMAKE_SOURCE_DIR}/applications/plugins/SofaROSConnector/ZySimulationAnalyzer)
include_directories(${CMAKE_SOURCE_DIR}/applications/plugins/SofaROSConnector/ZyROSConnector)
include_directories(${CMAKE_SOURCE_DIR}/applications/plugins/SofaROSConnector/ZyROSPublishingHandler)

add_library(${PROJECT_NAME} SHARED ${ZY_VELOCITYAPPROXIMATIONHANDLER_HEADER_FILES} ${ZY_VELOCITYAPPROXIMATIONHANDLER_SOURCE_FILES})

target_link_libraries(${PROJECT_NAME} SofaCore SofaHelper)
target_link_libraries(${PROJECT_NAME} ZySimulationAnalyzer)

set_target_properties(${PROJECT_NAME} PROPERTIES FOLDER "Plugins")
set_target_properties(${PROJECT_NAME} PROPERTIES DEBUG_POSTFIX "_d")

sofa_create_package(${PROJECT_NAME} "0.0.1" ${PROJECT_NAME} ${PROJECT_NAME})
