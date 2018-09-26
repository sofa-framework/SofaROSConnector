cmake_minimum_required(VERSION 2.8.12)
project(hashgen)

set(HEADER_FILES cryptohash.h)
set(SOURCE_FILES cryptohash.cpp)

add_library(${PROJECT_NAME} STATIC ${HEADER_FILES} ${SOURCE_FILES})
set_target_properties(${PROJECT_NAME} PROPERTIES FOLDER "SofaExtlibs")
target_include_directories(${PROJECT_NAME} PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>")
target_include_directories(${PROJECT_NAME} PUBLIC "$<INSTALL_INTERFACE:include>")
set_target_properties(${PROJECT_NAME} PROPERTIES PUBLIC_HEADER "${HEADER_FILES}")

# The code must be relocatable if we want to link a shared library against it
if("x${CMAKE_CXX_COMPILER_ID}" STREQUAL "xGNU" OR "x${CMAKE_CXX_COMPILER_ID}" STREQUAL "xClang")
    target_compile_options(${PROJECT_NAME} PRIVATE "-fPIC")
endif()

sofa_create_package(hashgen "1.0.0" hashgen hashgen)
