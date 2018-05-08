project(ZyGrippingHandler)

add_definitions(-DBUILD_ZYSOFA_GRIPPINGHANDLER)

set(ZY_VELOCITYAPPROXIMATIONHANDLER_HEADER_FILES
        init_ZyGrippingHandler.h
        ZyGrippingHandler.h)

set(ZY_VELOCITYAPPROXIMATIONHANDLER_SOURCE_FILES
        ZyGrippingHandler.cpp)

include_directories(${CMAKE_SOURCE_DIR}/SofaKernel/modules
                    ${CMAKE_SOURCE_DIR}/modules)
include_directories(${CMAKE_SOURCE_DIR}/applications/plugins/SofaROSConnector/ZyGripping)

# ROS (hydro)
include_directories(${ROS_INCLUDE_DIR})
include_directories(${LOG4CXX_INCLUDE_DIRS})

add_library(${PROJECT_NAME} SHARED ${ZY_VELOCITYAPPROXIMATIONHANDLER_HEADER_FILES} ${ZY_VELOCITYAPPROXIMATIONHANDLER_SOURCE_FILES})

target_link_libraries(${PROJECT_NAME} SofaCore SofaHelper)
target_link_libraries(${PROJECT_NAME} ZyGripping)

set_target_properties(${PROJECT_NAME} PROPERTIES FOLDER "Plugins")

sofa_create_package(${PROJECT_NAME} "0.0.1" ${PROJECT_NAME} ${PROJECT_NAME})
