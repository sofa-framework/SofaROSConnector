project(ZyVelocityApproximation)

set(HEADER_FILES
    initZyVelocityApproximation.h
    ZyVelocityApproximator.h
)

set(SOURCE_FILES
    initZyVelocityApproximation.cpp
    ZyVelocityApproximator.cpp
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
include_directories(${CMAKE_SOURCE_DIR})
include_directories(${CMAKE_SOURCE_DIR}/modules)

# ROS
include_directories(${ROS_INCLUDE_DIR})
include_directories(${LOG4CXX_INCLUDE_DIRS})

# Dependencies
include_directories(${CMAKE_SOURCE_DIR}/applications/plugins/SofaROSConnector/ZySOFAControllers)

add_definitions("-DSOFA_BUILD_ZY_VELOCITY_APPROXIMATION")
add_definitions(-DROS_BUILD_SHARED_LIBS)

add_library(${PROJECT_NAME} SHARED ${HEADER_FILES} ${SOURCE_FILES})

set_target_properties(${PROJECT_NAME} PROPERTIES FOLDER "Plugins")
set_target_properties(${PROJECT_NAME} PROPERTIES DEBUG_POSTFIX "_d")

target_link_libraries(${PROJECT_NAME} SofaCore SofaHelper SofaDefaultType SofaBaseCollision SofaConstraint SofaSimulationCommon)
target_link_libraries(${PROJECT_NAME} ZySOFAControllers)
target_link_libraries(${PROJECT_NAME} ${ROS_LIBRARY} ${ROS_CPP_LIBRARY} ${ROS_CPPCOMMON_LIBRARY} ${ROS_TIME_LIBRARY} ${ROS_CONSOLE_LIBRARY} ${ROS_CPP_SERIALIZATION_LIBRARY})

target_link_libraries(${PROJECT_NAME} ${Boost_REGEX_LIBRARY} ${Boost_THREAD_LIBRARY} ${Boost_SIGNALS_LIBRARY})
target_link_libraries(${PROJECT_NAME} ${LOG4CXX_LIBRARIES})

sofa_create_package(${PROJECT_NAME} "0.0.1" ${PROJECT_NAME} ${PROJECT_NAME})
