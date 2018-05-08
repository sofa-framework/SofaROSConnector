project(ZyROSPublishingHandler)

add_definitions(-DBUILD_SOFA_ZY_ROS_PUBLISHINGHANDLER)

set(TRU_THINGDOER_HEADER_FILES
        init_ZyROSPublishingHandler.h
        ZyROSPublishingHandler.h
	)

set(TRU_THINGDOER_SOURCE_FILES
        ZyROSPublishingHandler.cpp
	)

find_package(ROS REQUIRED)
message(STATUS "ROS found: ${ROS_FOUND}")
find_package(Boost COMPONENTS regex thread signals QUIET REQUIRED)
find_package(Log4cxx QUIET REQUIRED)

include_directories(${CMAKE_SOURCE_DIR}/applications/plugins/SofaROSConnector/ZyROSConnector)
include_directories(${CMAKE_SOURCE_DIR}/applications/plugins/SofaROSConnector/ZyROSConnectionManager)

# ROS
include_directories(${ROS_INCLUDE_DIR})
include_directories(${LOG4CXX_INCLUDE_DIRS})

add_library(${PROJECT_NAME} SHARED ${TRU_THINGDOER_HEADER_FILES} ${TRU_THINGDOER_SOURCE_FILES})

target_link_libraries(${PROJECT_NAME} SofaCore SofaHelper)
target_link_libraries(${PROJECT_NAME} ZyROSConnector)
target_link_libraries(${PROJECT_NAME} ZyROSConnectionManager)
target_link_libraries(${PROJECT_NAME} ${ROS_LIBRARY} ${ROS_CPP_LIBRARY} ${ROS_CPPCOMMON_LIBRARY} ${ROS_TIME_LIBRARY} ${ROS_CONSOLE_LIBRARY} ${ROS_CPP_SERIALIZATION_LIBRARY})

set_target_properties(${PROJECT_NAME} PROPERTIES FOLDER "TruLibs")

sofa_create_package(${PROJECT_NAME} "0.0.1" ${PROJECT_NAME} ${PROJECT_NAME})
