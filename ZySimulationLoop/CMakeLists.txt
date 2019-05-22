project(ZySimulationLoop)

set(HEADER_FILES
    initZySimulationLoop.h
    ZyPhysicsAnimationLoop.h
)

set(SOURCE_FILES
    initZySimulationLoop.cpp
    ZyPhysicsAnimationLoop.cpp
)


# SOFA include directories
include_directories(${CMAKE_CURRENT_SOURCE_DIR})
include_directories(${CMAKE_SOURCE_DIR})
include_directories(${CMAKE_SOURCE_DIR}/modules)

add_definitions("-DSOFA_BUILD_ZYSOFA_SIMULATION_LOOP")

add_library(${PROJECT_NAME} SHARED ${HEADER_FILES} ${SOURCE_FILES})

set_target_properties(${PROJECT_NAME} PROPERTIES FOLDER "Plugins")
set_target_properties(${PROJECT_NAME} PROPERTIES DEBUG_POSTFIX "_d")

target_link_libraries(${PROJECT_NAME} SofaCore SofaHelper SofaDefaultType SofaBaseCollision SofaConstraint SofaSimulationCommon SofaUserInteraction)

sofa_create_package(${PROJECT_NAME} "0.0.1" ${PROJECT_NAME} ${PROJECT_NAME})
