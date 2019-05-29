project(ZySimulationLoop)

set(HEADER_FILES
    initZySimulationLoop.h
    ZyPhysicsAnimationLoop.h
)

set(SOURCE_FILES
    initZySimulationLoop.cpp
    ZyPhysicsAnimationLoop.cpp
)

find_package(ROS QUIET REQUIRED)
message(STATUS "ROS found: ${ROS_FOUND}")
find_package(Boost COMPONENTS regex thread signals QUIET REQUIRED)
find_package(Log4cxx QUIET REQUIRED)

# Boost
find_package(Boost COMPONENTS system chrono thread REQUIRED QUIET)
include_directories(${Boost_INCLUDE_DIR})
link_directories(${Boost_LIBRARY_DIRS})

# ROS
include_directories(${ROS_INCLUDE_DIR})
include_directories(${LOG4CXX_INCLUDE_DIRS})

# SOFA include directories
include_directories(${CMAKE_CURRENT_SOURCE_DIR})
include_directories(${CMAKE_SOURCE_DIR})
include_directories(${CMAKE_SOURCE_DIR}/modules)

# ROS connector include directories
include_directories(${CMAKE_SOURCE_DIR}/applications/plugins/SofaROSConnector)
include_directories(${CMAKE_SOURCE_DIR}/applications/plugins/SofaROSConnector/ZyMultithreading)
include_directories(${CMAKE_SOURCE_DIR}/applications/plugins/SofaROSConnector/ZySOFAControllers)
include_directories(${CMAKE_SOURCE_DIR}/applications/plugins/SofaROSConnector/ZyROSConnector)
include_directories(${CMAKE_SOURCE_DIR}/applications/plugins/SofaROSConnector/ZyROSPublishingHandler)
include_directories(${CMAKE_SOURCE_DIR}/applications/plugins/SofaROSConnector/ZyGripping)
include_directories(${CMAKE_SOURCE_DIR}/applications/plugins/SofaROSConnector/ZyGrippingHandler)
include_directories(${CMAKE_SOURCE_DIR}/applications/plugins/SofaROSConnector/ZySimulationAnalyzer)
include_directories(${CMAKE_SOURCE_DIR}/applications/plugins/SofaROSConnector/ZySimulationAnalyzerHandler)
include_directories(${CMAKE_SOURCE_DIR}/applications/plugins/SofaROSConnector/ZyVelocityApproximation)
include_directories(${CMAKE_SOURCE_DIR}/applications/plugins/SofaROSConnector/ZyVelocityApproximationHandler)

add_definitions("-DSOFA_BUILD_ZYSOFA_SIMULATION_LOOP")

add_library(${PROJECT_NAME} SHARED ${HEADER_FILES} ${SOURCE_FILES})

set_target_properties(${PROJECT_NAME} PROPERTIES FOLDER "Plugins")
set_target_properties(${PROJECT_NAME} PROPERTIES DEBUG_POSTFIX "_d")

target_link_libraries(${PROJECT_NAME} SofaCore SofaHelper SofaDefaultType SofaBaseCollision SofaConstraint SofaSimulationCommon SofaUserInteraction)

sofa_create_package(${PROJECT_NAME} "0.0.1" ${PROJECT_NAME} ${PROJECT_NAME})
