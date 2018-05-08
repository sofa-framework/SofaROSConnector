cmake_minimum_required(VERSION 2.8.12)
project(ZyMultiThreading)

set(TRU_MULTITHREADING_MAJOR_VERSION 0)
set(TRU_MULTITHREADING_MINOR_VERSION 1)
set(TRU_MULTITHREADING_VERSION ${TRU_MULTITHREADING_MAJOR_VERSION}.${TRU_MULTITHREADING_MINOR_VERSION})

find_package(Boost COMPONENTS system chrono thread REQUIRED QUIET)
include_directories(${Boost_INCLUDE_DIR})
link_directories(${Boost_LIBRARY_DIRS})

set(HEADER_FILES config_multithreading.h
				 WorkerThreadIface.h
				 WorkerThread_Pool.h
				 WorkerThread_Pool.inl
				 WorkerThread_SingleTask.h
				 TaskScheduler.h
				 TaskSchedulerBoostPool.h
				 TaskSchedulerBoostPool.inl
				 MultiThread_Scheduler.h
				 Tasks.h
				 Tasks.inl)

set(SOURCE_FILES WorkerThreadIface.cpp
				 WorkerThread_Pool.cpp
				 WorkerThread_SingleTask.cpp
				 TaskScheduler.cpp
				 TaskSchedulerBoostPool.cpp
				 MultiThread_Scheduler.cpp
				 Tasks.cpp)
	
add_definitions(-DBUILD_TRU_MULTITHREADING)

add_library(${PROJECT_NAME} SHARED ${HEADER_FILES} ${SOURCE_FILES})

target_link_libraries(${PROJECT_NAME} SofaBaseMechanics SofaMiscMapping)
target_link_libraries(${PROJECT_NAME} ${Boost_SYSTEM_LIBRARY} ${Boost_THREAD_LIBRARY} ${Boost_CHRONO_LIBRARY})
if (UNIX)
  target_link_libraries(${PROJECT_NAME} pthread)
endif(UNIX)

target_include_directories(${PROJECT_NAME} PUBLIC "$<BUILD_INTERFACE:${CMAKE_BINARY_DIR}/include>")
target_include_directories(${PROJECT_NAME} PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/..>")
target_include_directories(${PROJECT_NAME} PUBLIC "$<INSTALL_INTERFACE:include>")
set_target_properties(${PROJECT_NAME} PROPERTIES COMPILE_FLAGS "-DBUILD_TRU_MULTITHREADING")
set_target_properties(${PROJECT_NAME} PROPERTIES VERSION ${TRU_MULTITHREADING_VERSION})
set_target_properties(${PROJECT_NAME} PROPERTIES PUBLIC_HEADER "${HEADER_FILES}")
include_directories("${CMAKE_CURRENT_SOURCE_DIR}")

set_target_properties(${PROJECT_NAME} PROPERTIES FOLDER "TruLibs")

sofa_create_package(ZyMultiThreading ${TRU_MULTITHREADING_VERSION} ZyMultiThreading ZyMultiThreading)


