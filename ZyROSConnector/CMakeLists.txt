cmake_minimum_required(VERSION 2.8.12)
project(ZyROSConnector)

set(TRU_ROSCONNECTOR_MAJOR_VERSION 0)
set(TRU_ROSCONNECTOR_MINOR_VERSION 2)
set(TRU_ROSCONNECTOR_VERSION ${TRU_ROSCONNECTOR_MAJOR_VERSION}.${TRU_ROSCONNECTOR_MINOR_VERSION})

set(HEADER_FILES
        init_ZyROSConnector.h
        ZyROSConnector.h
        ZyROSConnectorWorkerThread.h
        ZyROSConnectorTopicSubscriber.h
        ZyROSConnectorTopicSubscriber.inl
        ZyROSConnectorTopicPublisher.h
        ZyROSConnectorTopicPublisher.inl
        ZyROSConnectorServiceClient.h
        ZyROSConnectorServiceClient.inl
        ZyROSConnectorServiceServer.h
        ZyROSConnectorServiceServer.inl

        # ZyROSTopicListener_Bool.h
        # ZyROSTopicListener_JointState.h
        # ZyROSTopicListener_LogMsgs.h
        # ZyROSTopicPublisher_Float32.h
        # ZyROSTopicPublisher_Float32MultiArray.h
)

set(SOURCE_FILES
        ZyROSConnector.cpp
        ZyROSConnectorWorkerThread.cpp
        ZyROSConnectorTopicSubscriber.cpp
        ZyROSConnectorTopicPublisher.cpp
        ZyROSConnectorServiceClient.cpp
        ZyROSConnectorServiceServer.cpp

        # ZyROSTopicListener_Bool.cpp
        # ZyROSTopicListener_JointState.cpp
        # ZyROSTopicListener_LogMsgs.cpp
        # ZyROSTopicPublisher_Float32.cpp
        # ZyROSTopicPublisher_Float32MultiArray.cpp
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

# Dependency libraries
include_directories(${CMAKE_SOURCE_DIR}/applications/plugins/SofaROSConnector)
include_directories(${CMAKE_SOURCE_DIR}/applications/plugins/SofaROSConnector/ZyGripping)
include_directories(${CMAKE_SOURCE_DIR}/applications/plugins/SofaROSConnector/ZySOFAControllers)
include_directories(${CMAKE_SOURCE_DIR}/applications/plugins/SofaROSConnector/ZyVelocityApproximation)

# ROS
include_directories(${ROS_INCLUDE_DIR})
include_directories(${LOG4CXX_INCLUDE_DIRS})

add_definitions("-DSOFA_BUILD_ZY_ROS_CONNECTOR")
add_definitions(-DROS_BUILD_SHARED_LIBS)

add_library(${PROJECT_NAME} SHARED ${HEADER_FILES} ${SOURCE_FILES})

target_link_libraries(${PROJECT_NAME} SofaCore SofaHelper SofaDefaultType SofaBaseCollision SofaConstraint SofaSimulationCommon SofaMiscFem tinyxml newmat)
target_link_libraries(${PROJECT_NAME} ZyMultiThreading)
target_link_libraries(${PROJECT_NAME} ZyGripping)
target_link_libraries(${PROJECT_NAME} ZySOFAControllers)
target_link_libraries(${PROJECT_NAME} ZyVelocityApproximation)
target_link_libraries(${PROJECT_NAME} ${ROS_LIBRARY} ${ROS_CPP_LIBRARY} ${ROS_CPPCOMMON_LIBRARY} ${ROS_TIME_LIBRARY} ${ROS_CONSOLE_LIBRARY} ${ROS_CPP_SERIALIZATION_LIBRARY})

target_link_libraries(${PROJECT_NAME} ${Boost_REGEX_LIBRARY} ${Boost_THREAD_LIBRARY} ${Boost_SIGNALS_LIBRARY})
target_link_libraries(${PROJECT_NAME} ${LOG4CXX_LIBRARIES})

set_target_properties(${PROJECT_NAME} PROPERTIES FOLDER "Plugins")
set_target_properties(${PROJECT_NAME} PROPERTIES DEBUG_POSTFIX "_d")

sofa_create_package(${PROJECT_NAME} "0.0.1" ${PROJECT_NAME} ${PROJECT_NAME})
