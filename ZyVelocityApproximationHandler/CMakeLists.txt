project(ZyVelocityApproximationHandler)

add_definitions(-DSOFA_BUILD_ZY_VELOCITY_APPROXIMATION_HANDLER)

set(ZY_VELOCITYAPPROXIMATIONHANDLER_HEADER_FILES
        init_ZyVelocityApproximationHandler.h
        ZyVelocityApproximationHandler.h)

set(ZY_VELOCITYAPPROXIMATIONHANDLER_SOURCE_FILES
        ZyVelocityApproximationHandler.cpp)

include_directories(${CMAKE_SOURCE_DIR}/SofaKernel/modules
                    ${CMAKE_SOURCE_DIR}/modules)
include_directories(${CMAKE_SOURCE_DIR}/applications/plugins/SofaROSConnector/ZyVelocityApproximation)

# ROS (hydro)
include_directories(${ROS_INCLUDE_DIR})
include_directories(${LOG4CXX_INCLUDE_DIRS})

include_directories(${CMAKE_SOURCE_DIR}/applications/plugins/SofaROSConnector/ZySOFAControllers)

add_library(${PROJECT_NAME} SHARED ${ZY_VELOCITYAPPROXIMATIONHANDLER_HEADER_FILES} ${ZY_VELOCITYAPPROXIMATIONHANDLER_SOURCE_FILES})

target_link_libraries(${PROJECT_NAME} SofaCore SofaHelper)
target_link_libraries(${PROJECT_NAME} ZyVelocityApproximation)
target_link_libraries(${PROJECT_NAME} ZySOFAControllers)

set_target_properties(${PROJECT_NAME} PROPERTIES FOLDER "Plugins")
set_target_properties(${PROJECT_NAME} PROPERTIES DEBUG_POSTFIX "_d")

sofa_create_package(${PROJECT_NAME} "0.0.1" ${PROJECT_NAME} ${PROJECT_NAME})
