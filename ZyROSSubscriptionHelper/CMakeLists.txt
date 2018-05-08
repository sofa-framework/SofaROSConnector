project(ZyROSSubscriptionHelper)

add_definitions(-DSOFA_BUILD_ZY_ROS_SUBSCRIPTIONHELPER)

set(TRU_THINGDOER_HEADER_FILES
        init_ZyROSSubscriptionHelper.h
        ZyROSSubscriptionHelper.h)

set(TRU_THINGDOER_SOURCE_FILES
        ZyROSSubscriptionHelper.cpp)

find_package(ROS QUIET REQUIRED)
message(STATUS "ROS found: ${ROS_FOUND}")
find_package(Boost COMPONENTS regex thread signals QUIET REQUIRED)
find_package(Log4cxx QUIET REQUIRED)

include_directories(${CMAKE_SOURCE_DIR}/applications/plugins/SofaROSConnector/ZyROSConnector)
include_directories(${CMAKE_SOURCE_DIR}/applications/plugins/SofaROSConnector/ZyROSConnectionManager)
include_directories(${CMAKE_SOURCE_DIR}/applications/plugins/SofaROSConnector/ZySimulationAnalyzer)
include_directories(${CMAKE_SOURCE_DIR}/applications/plugins/SofaROSConnector/ZyROSPublishingHandler)

# ROS
include_directories(${ROS_INCLUDE_DIR})
include_directories(${LOG4CXX_INCLUDE_DIRS})

add_library(${PROJECT_NAME} SHARED ${TRU_THINGDOER_HEADER_FILES} ${TRU_THINGDOER_SOURCE_FILES})

target_link_libraries(${PROJECT_NAME} SofaCore SofaHelper)
target_link_libraries(${PROJECT_NAME} ZyROSConnector ZySimulationAnalyzer)
target_link_libraries(${PROJECT_NAME} ${ROS_LIBRARY} ${ROS_CPP_LIBRARY} ${ROS_CPPCOMMON_LIBRARY} ${ROS_TIME_LIBRARY} ${ROS_CONSOLE_LIBRARY} ${ROS_CPP_SERIALIZATION_LIBRARY})

set_target_properties(${PROJECT_NAME} PROPERTIES FOLDER "Plugins")

sofa_create_package(${PROJECT_NAME} "0.0.1" ${PROJECT_NAME} ${PROJECT_NAME})
