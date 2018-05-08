cmake_minimum_required(VERSION 2.8.12)

project(TruRosConnector_test)

set(SOURCE_FILES
    TruRosConnectorTest.cpp
)

add_definitions(-DQT_QTWINMIGRATE_IMPORT)

add_executable(TruRosConnector_test ${SOURCE_FILES})
target_link_libraries(${PROJECT_NAME} TruRosConnector SofaTest SofaGTestMain)
target_link_libraries(${PROJECT_NAME} ${QT_QTCORE_LIBRARY})

add_test(NAME ${PROJECT_NAME} COMMAND ${PROJECT_NAME})
