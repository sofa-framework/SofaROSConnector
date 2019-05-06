cmake_minimum_required(VERSION 2.8.12)

project(ZyROSConnector_test)

set(SOURCE_FILES
    ZyROSConnectorTest.cpp
)

add_definitions(-DQT_QTWINMIGRATE_IMPORT)

add_executable(${PROJECT_NAME} ${SOURCE_FILES})
target_link_libraries(${PROJECT_NAME} ZyROSConnector ZyROSConnectionManager SofaTest SofaGTestMain)
target_link_libraries(${PROJECT_NAME} ${QT_QTCORE_LIBRARY})

add_test(NAME ${PROJECT_NAME} COMMAND ${PROJECT_NAME})
